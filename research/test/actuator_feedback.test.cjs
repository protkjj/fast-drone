const {test,before,after}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),R=require('../runtime.js');
let ca,data,b;
before(async()=>{
  ca=await require('@casadi/casadi-wasm')();
  await ca.load_nlpsol('ipopt');await ca.load_interpolant('linear');
  data=JSON.parse(fs.readFileSync(path.join(__dirname,'../generated/selected.json')));
  b=R.makeBindings(ca,data);
});
after(()=>b?.dispose());
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const near=(a,b,tol=1e-7)=>assert.ok(Math.abs(a-b)<tol,`${a} != ${b}`);

test('actuator forecast matches the Python fixture and respects current/voltage-dependent response',()=>{
  const fixture=data.parity;
  const [step]=b.call(b.functions.motor_step,fixture.x.slice(13,17),fixture.u,[2],[22]);
  step.forEach((v,i)=>near(v,fixture.motor_step[i]));
  const indi=new R.INDI(b,data),state=data.initial.slice(0,17),snapshot=state.slice();
  const high=indi.envelope(state,25.2),low=indi.envelope(state,10);
  assert.ok(high.available&&low.available);
  assert.equal(high.horizon_s,.02);
  assert.ok(low.force_upper_N.every((v,i)=>v<high.force_upper_N[i]));
  // Passive coasting cannot remove all existing rotor thrust within 20 ms.
  assert.ok(high.force_lower_N.every(v=>v>0));
  assert.ok(high.rotor_upper_rad_s.every(v=>v<indi.max));
  assert.deepEqual(state,snapshot);
  assert.throws(()=>indi.envelope(state,NaN),/Invalid/);
  // In the current-limited regime, a voltage decrease need not reduce authority.
  assert.deepEqual(indi.envelope(state,15).force_upper_N,high.force_upper_N);
  const fast=state.slice();for(let j=13;j<17;j++)fast[j]=3000;
  assert.ok(indi.envelope(fast,18).force_upper_N.every((v,i)=>v<indi.envelope(fast,25.2).force_upper_N[i]));
});

test('motor endpoint intervals contain sampled commands and expose coupled, not independent, virtual limits',()=>{
  const indi=new R.INDI(b,data),state=data.initial.slice(0,17),envelope=indi.envelope(state,22);
  const {matrix,inverse,offset,force_lower_N:lo,force_upper_N:hi}=envelope;
  for(let sample=0;sample<=10;sample++){
    const command=Array.from({length:4},(_,i)=>indi.max*((sample+3*i)%11)/10);
    let n=state.slice(13,17);
    for(let k=0;k<20;k++)[n]=b.call(b.functions.motor_step,n,command,[0],[22]);
    const [force]=b.call(b.functions.rotors,n,[0]);
    force.forEach((v,i)=>assert.ok(v>=lo[i]-1e-7&&v<=hi[i]+1e-7));
  }
  const force=lo.map((v,i)=>(v+hi[i])/2);
  const virtual=matrix.map((row,i)=>dot(row,force)+offset[i]);
  inverse.forEach((row,i)=>near(dot(row,virtual.map((v,k)=>v-offset[k])),force[i]));
  // At maximum collective force there is no independent room for max pitch.
  const impossible=matrix.map((row,i)=>row.reduce((s,v,k)=>s+v*(v>=0?hi[k]:lo[k]),offset[i]));
  const inferred=inverse.map(row=>dot(row,impossible.map((v,i)=>v-offset[i])));
  assert.ok(inferred.some((v,i)=>v<lo[i]-1e-6||v>hi[i]+1e-6));
});

test('envelope reads observation state, not hidden SOC/plant error; zero RPM and axial inflow remain finite',()=>{
  const indi=new R.INDI(b,data),x=data.initial.slice();
  const original=indi.envelope(x,22);x[17]=.1;
  assert.deepEqual(indi.envelope(x,22),original);
  x[13]=0;x[14]=0;x[15]=0;x[16]=0;
  assert.ok(indi.envelope(x,22).available);
  const moving=data.initial.slice();moving[5]=20;
  const envelope=indi.envelope(moving,22);
  near(envelope.axial_mps,20);
  assert.ok(envelope.available);
  assert.ok(Number.isFinite(envelope.roll_secant_midpoint_error_rad_s2));
});

test('actual IPOPT enforces the first-move coupled envelope and keeps 13 states',()=>{
  const indi=new R.INDI(b,data),optimizer=new R.Optimizer(ca,b,data,'hybrid');
  const envelope=indi.envelope(data.initial,22);
  const refs=Array.from({length:21},()=>[3,0,0,21]);
  const command=optimizer.solve(data.initial,refs,22,[0,0,0],envelope);
  const stats=optimizer.stats[0];
  assert.equal(optimizer.meta.nx,13);assert.ok(stats.success,JSON.stringify(stats));
  assert.ok(stats.envelope_violation_N<1e-3);
  assert.ok(command.every(Number.isFinite));
  optimizer.solve(data.initial,refs,22,[0,0,0],envelope);
  assert.ok(optimizer.lamG.every(Number.isFinite));
  assert.equal(optimizer.lamG.length,optimizer.meta.lbg.length);
});

test('runtime passes capability information at 50 Hz and provides an explicit old-Hybrid ablation',async()=>{
  const original=R.Optimizer.prototype.solve,seen=[];
  R.Optimizer.prototype.solve=function(state,refs,voltage,wind,envelope){
    seen.push(envelope);this.stats.push({ms:0,success:true,status:'test',iterations:0,residual:0});
    return this.previous.slice();
  };
  try{
    await R.run(ca,data,{controller:'hybrid',scenario:'hover',seconds:.04,hybrid_actuator_feedback:true});
    assert.equal(seen.length,2);assert.ok(seen.every(e=>e.available));
    near(seen[0].t_s,0);near(seen[1].t_s,.02);
    seen.length=0;
    const old=await R.run(ca,data,{controller:'hybrid',scenario:'hover',seconds:.04});
    assert.ok(seen.every(e=>e===null));assert.equal(old.configuration.hybrid_actuator_feedback,false);
    seen.length=0;
    await R.run(ca,data,{controller:'nmpc',scenario:'hover',seconds:.02});
    assert.deepEqual(seen,[null]);
  }finally{R.Optimizer.prototype.solve=original;}
});

test('zero electrical headroom retains the passive-coasting equality',()=>{
  const indi=new R.INDI(b,data),envelope=indi.envelope(data.initial,5);
  assert.ok(envelope.available);
  envelope.force_lower_N.forEach((v,i)=>near(v,envelope.force_upper_N[i]));
});

test('failure does not silently project the held command into a new feasible envelope',()=>{
  const indi=new R.INDI(b,data),optimizer=new R.Optimizer(ca,b,data,'hybrid');
  const envelope=indi.envelope(data.initial,7),held=optimizer.command.slice();
  optimizer.solver={call(){throw new Error('deliberate failure');}};
  const result=optimizer.solve(data.initial,Array.from({length:21},()=>[0,0,0,20]),7,[0,0,0],envelope);
  assert.deepEqual(result,held);assert.equal(optimizer.stats[0].success,false);
  assert.ok(optimizer.stats[0].envelope_violation_N>0);
});
