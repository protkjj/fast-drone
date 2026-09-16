const {test,before,after}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const runtime=require('../runtime.js');
const stl=require('../stl.js');
let ca,data,b;
const near=(a,b,tolerance=1e-8)=>assert.ok(Math.abs(a-b)<=tolerance,`${a} != ${b}`);
before(async()=>{
  ca=await require('@casadi/casadi-wasm')();
  await ca.load_nlpsol('ipopt');await ca.load_interpolant('linear');
  data=JSON.parse(fs.readFileSync(path.join(__dirname,'../generated/simple.json'),'utf8'));
  b=runtime.makeBindings(ca,data,Object.keys(data.solvers),false);
});
after(()=>b?.dispose());
test('20 ms warm-start shift interpolates the 50 ms grid without mutating the source',()=>{
  const values=[0,10,50,60,100,110];
  assert.deepEqual(runtime.shiftPrediction(values,2,.4),[20,30,70,80,100,110]);
  assert.deepEqual(values,[0,10,50,60,100,110]);
  assert.deepEqual(runtime.shiftPrediction([],160,.4),[]);
});
test('Python and WASM evaluate identical plant RHS and RK4 step',()=>{
  const p=data.parity;
  for(const key of ['rhs','step']) {
    const result=b.call(b.functions[key],p.x,p.u,p.env,p.scales)[0];
    result.forEach((v,i)=>near(v,p[key][i],1e-8));
  }
});
test('bounded INDI allocation respects all four actuator constraints',()=>{
  const matrix=[[1,1,1,1],[1,-1,1,-1],[1,1,-1,-1],[-1,1,1,-1]];
  const u=runtime.boundedIncrement(matrix,[4,10,0,0],[0,0,0,0],[1,1,1,1],[1,1,1,1]);
  assert.ok(u.every(v=>v>=-1e-7&&v<=1+1e-7));
  near(u[0],1);near(u[2],1);near(u[1],0);near(u[3],0);
});
test('estimated bias corrections are not mistaken for measured angular acceleration',()=>{
  const indi=new runtime.INDI(b,data),state=data.initial.slice(0,17);
  const measured=[.005,-.003,.002];indi.gyro=measured.slice();
  state[10]=-.01; // A changed estimated gyro bias, with no measured rotation change.
  indi.update(state,data.solvers.hybrid.hover,measured);
  indi.alpha.forEach(v=>near(v,0,1e-12));
});
test('delayed GPS correction replays IMU and matches an on-time update',()=>{
  const immediate=new runtime.Estimator(b,data,data.initial),delayed=new runtime.Estimator(b,data,data.initial);
  const imu=[data.profile.g,0,0,0,0,0],gps=[.1,-.2,20.3,.01,-.01,0];
  for(let tick=0;tick<120;tick++) {
    immediate.predict(imu);delayed.predict(imu);
    if(tick===99) immediate.gps(100,gps);
  }
  delayed.gps(100,gps);
  immediate.state.forEach((v,i)=>near(v,delayed.state[i],1e-12));
  immediate.cov.forEach((v,i)=>near(v,delayed.cov[i],1e-12));
  assert.equal(delayed.replayed,20);
});
test('a 40 ms Hybrid run has exactly 2 / 40 / 40 NMPC, INDI and IMU samples',async()=>{
  const result=await runtime.run(ca,data,{seconds:.04,scenario:'hover',controller:'hybrid',feedback:'truth'});
  assert.equal(result.failure,null);assert.equal(result.metrics.solver_failures,0);
  assert.equal(result.counts.nmpc,2);assert.equal(result.counts.indi,40);assert.equal(result.counts.imu,40);
  assert.equal(result.counts.plant,40);assert.ok(result.metrics.velocity_rmse_mps<1e-5);
  assert.ok(Number.isFinite(result.metrics.solve_ms_max));
  assert.equal(result.solves.length,2);
});
test('standalone NMPC never executes INDI',async()=>{
  const result=await runtime.run(ca,data,{seconds:.02,scenario:'hover',controller:'nmpc',feedback:'truth'});
  assert.equal(result.counts.indi,0);assert.equal(result.counts.nmpc,1);
  assert.equal(result.metrics.solver_failures,0);assert.ok(result.metrics.velocity_rmse_mps<1e-4);
});
test('selected-aircraft WASM parity and both controllers survive a sequential hover run',async()=>{
  const selected=JSON.parse(fs.readFileSync(path.join(__dirname,'../generated/selected.json'),'utf8'));
  const bindings=runtime.makeBindings(ca,selected,Object.keys(selected.solvers),false);
  try {
    const p=selected.parity;
    for(const key of ['rhs','step']) {
      const result=bindings.call(bindings.functions[key],p.x,p.u,p.env,p.scales)[0];
      result.forEach((v,i)=>near(v,p[key][i],1e-8));
    }
  } finally {bindings.dispose();}
  for(const controller of ['hybrid','nmpc']) {
    const result=await runtime.run(ca,selected,{seconds:.02,scenario:'hover',controller,feedback:'truth'});
    assert.equal(result.failure,null);assert.equal(result.metrics.solver_failures,0);
    assert.equal(result.counts.nmpc,1);assert.ok(result.metrics.velocity_rmse_mps<1e-4);
  }
});
test('selected standalone NMPC solves the first speed-preview problem within 30 iterations',async()=>{
  const selected=JSON.parse(fs.readFileSync(path.join(__dirname,'../generated/selected.json'),'utf8'));
  // This is one optimizer problem, not a complete speed-step experiment.
  const bindings=runtime.makeBindings(ca,selected);
  try {
    const optimizer=new runtime.Optimizer(ca,bindings,selected,'nmpc');
    optimizer.solve(selected.initial,runtime.referenceHorizon(0,{scenario:'step',speed:3}),
      selected.profile.battery.series*4.2,[0,0,0]);
    assert.ok(optimizer.stats[0].success,JSON.stringify(optimizer.stats));
    assert.ok(optimizer.stats[0].iterations<=30);assert.ok(optimizer.stats[0].residual<1e-3);
  }finally{bindings.dispose();}
});
test('STL uses metres and the selected CG, with body +x pointing at the nose',()=>{
  const bytes=fs.readFileSync(path.join(__dirname,'../assets/drone_v2.stl'));
  const buffer=bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength);
  const mesh=stl.parse(buffer);
  assert.equal(mesh.triangles,173804);
  near(mesh.max[0],.4282511278030152);
  near(mesh.max[0]-mesh.min[0],.6704993286132812);
  assert.equal(mesh.positions.length,173804*9);
});
test('invalid STL and invalid run settings fail explicitly',async()=>{
  assert.throws(()=>stl.parse(new ArrayBuffer(12)),/incomplete/);
  await assert.rejects(runtime.run(ca,data,{seconds:NaN}),/Duration/);
  await assert.rejects(runtime.run(ca,data,{controller:'fake'}),/controller/);
});
