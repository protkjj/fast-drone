const {test,before,after}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const R=require('../runtime.js');
let ca,data,b;
before(async()=>{
  ca=await require('@casadi/casadi-wasm')();
  await ca.load_nlpsol('ipopt');await ca.load_interpolant('linear');
  data=JSON.parse(fs.readFileSync(path.join(__dirname,'../generated/selected.json')));
  b=R.makeBindings(ca,data);
});
after(()=>b?.dispose());
const near=(a,c,tol=1e-5)=>assert.ok(Math.abs(a-c)<tol,`${a} != ${c}`);

test('INDI cannot create commanded thrust to satisfy an infeasible angular request',()=>{
  const indi=new R.INDI(b,data);
  const request=[0,100,100,100],state=data.initial.slice(0,17),before=state.slice();
  const command=indi.update(state,request,[0,0,0]);
  const [wrench]=b.call(b.functions.effect,command,[0]);
  near(wrench[0],0);assert.deepEqual(state,before);
  assert.deepEqual(indi.last.requested,request);
  near(indi.last.allocated[0],0);
  assert.ok(Math.hypot(...indi.last.residual.slice(1))>100);
});

test('nonlinear allocation preserves feasible total thrust under angular saturation',()=>{
  const weight=data.profile.mass_kg*data.profile.g;
  for(const total of [weight*.2,weight,weight*1.2]){
    const indi=new R.INDI(b,data);
    const command=indi.update(data.initial.slice(0,17),[total,100,100,100],[0,0,0]);
    const [wrench]=b.call(b.functions.effect,command,[0]);
    near(wrench[0],total,1e-4);
    assert.ok(command.every(n=>n>=0&&n<=indi.max));
    near(indi.last.requested[0]-indi.last.allocated[0],indi.last.residual[0]);
  }
});

test('motor diagnostic distinguishes electrical limiting from command bounds',()=>{
  const x=data.initial.slice(),u=x.slice(13,17).map(n=>n*1.3);
  assert.ok(u.every(n=>n<data.solvers.nmpc.max_rotor_rad_s));
  const d=b.call(b.functions.diag,x,u,[0,0,0,0,0,0],[1,1,1,1,1]);
  const channels=R.motorDiagnostics(d);
  assert.ok(channels.current_limited.some(Boolean));
  assert.ok(channels.tracking_limited.some(Boolean));
});

test('scenario validation rejects experiments that never reach their event',()=>{
  assert.throws(()=>R.validateOptions({scenario:'step',seconds:.2}),/1/);
  assert.throws(()=>R.validateOptions({scenario:'gust',seconds:2}),/3/);
  assert.equal(R.validateOptions({scenario:'gust',seconds:4}).seconds,4);
  const cfg={scenario:'step',speed:3,preview:false};
  assert.ok(R.referenceHorizon(.2,cfg).every(r=>r[0]===0));
  assert.ok(R.referenceHorizon(1.1,cfg).every(r=>r[0]===3));
});

test('stop is checked before starting an optimizer and after each solve',async()=>{
  const original=R.Optimizer.prototype.solve;
  let calls=0,stop=false;
  R.Optimizer.prototype.solve=function(){
    calls++;stop=true;this.stats.push({ms:0,success:true,status:'test',iterations:0,residual:0});
    return this.previous.slice();
  };
  try{
    const never=await R.run(ca,data,{controller:'nmpc',scenario:'hover',seconds:.2},()=>{},()=>true);
    assert.equal(calls,0);assert.equal(never.counts.plant,0);
    const one=await R.run(ca,data,{controller:'nmpc',scenario:'hover',seconds:.2},()=>{},()=>stop);
    assert.equal(calls,1);assert.equal(one.counts.nmpc,1);assert.equal(one.counts.plant,0);
  }finally{R.Optimizer.prototype.solve=original;}
});

test('recorded state and sensor channels have explicit common capture times',async()=>{
  const original=R.Optimizer.prototype.solve;
  R.Optimizer.prototype.solve=function(){
    this.stats.push({ms:0,success:true,status:'test',iterations:0,residual:0});return this.previous.slice();
  };
  try{
    const r=await R.run(ca,data,{controller:'nmpc',scenario:'hover',seconds:.04,feedback:'eskf'});
    assert.equal(r.trace[0].t,0);near(r.trace.at(-1).t,.04);
    for(const row of r.trace){
      assert.equal(row.state.length,18);assert.equal(row.body_rate_rad_s.length,3);
      assert.equal(row.rotor_rad_s.length,4);assert.equal(row.motor.current_A.length,4);
      near(row.sensor.capture_t_s,row.t);
      near(row.sensor.gyro_t_s,row.sensor.rotor_t_s);
      if(row.sensor.imu)near(row.sensor.imu_t_s,row.t);
    }
  }finally{R.Optimizer.prototype.solve=original;}
});

test('Hybrid passes achieved nominal allocation into the next delta-input cost',async()=>{
  const original=R.Optimizer.prototype.solve,previous=[];
  R.Optimizer.prototype.solve=function(){
    previous.push(this.previous.slice());this.stats.push({ms:0,success:true,status:'test',iterations:0,residual:0});
    return [0,100,100,100];
  };
  try{
    const r=await R.run(ca,data,{controller:'hybrid',scenario:'hover',seconds:.04});
    assert.equal(r.counts.nmpc,2);near(previous[1][0],0);
    assert.ok(Math.hypot(...previous[1].slice(1))<1e-5);
    assert.deepEqual(r.trace[0].allocation.requested,[0,100,100,100]);
  }finally{R.Optimizer.prototype.solve=original;}
});

test('optimizer failure holds the bounded command, not external allocation feedback',()=>{
  const optimizer=new R.Optimizer(ca,b,data,'hybrid'),held=optimizer.command.slice();
  optimizer.previous=[999,999,999,999];
  optimizer.solver={call(){throw new Error('deliberate test failure');}};
  assert.deepEqual(optimizer.solve(data.initial,R.referenceHorizon(0,{scenario:'hover'}),25.2,[0,0,0]),held);
  assert.equal(optimizer.stats[0].success,false);
});

test('log sampling rate does not change dynamics, noise streams or integrated metrics',async()=>{
  const original=R.Optimizer.prototype.solve;
  R.Optimizer.prototype.solve=function(){
    this.stats.push({ms:0,success:true,status:'test',iterations:0,residual:0});return this.previous.slice();
  };
  try{
    const options={controller:'nmpc',feedback:'eskf',scenario:'hover',seconds:.04,altitude:33};
    const low=await R.run(ca,data,{...options,log_hz:50}),high=await R.run(ca,data,{...options,log_hz:1000});
    assert.deepEqual(low.final,high.final);assert.deepEqual(low.metrics,high.metrics);
    assert.equal(low.trace.length,3);assert.equal(high.trace.length,41);
    assert.equal(low.trace[0].state[2],33);assert.equal(low.trace[0].reference[3],33);
  }finally{R.Optimizer.prototype.solve=original;}
});
