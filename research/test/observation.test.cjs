const {test,before}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path');
const R=require('../runtime.js'),Results=require('../results.js');
let ca,data;
before(async()=>{
  ca=await require('@casadi/casadi-wasm')();
  await ca.load_nlpsol('ipopt');await ca.load_interpolant('linear');
  data=JSON.parse(fs.readFileSync(path.join(__dirname,'../generated/selected.json')));
});
test('observation PD–INDI uses common plant and never runs an NMPC solver',async()=>{
  const r=await R.run(ca,data,{controller:'pd',scenario:'hover',seconds:.04});
  assert.equal(r.counts.nmpc,0);assert.equal(r.counts.outer,2);
  assert.equal(r.counts.indi,40);assert.equal(r.counts.plant,40);assert.equal(r.failure,null);
  assert.equal(r.solver,'PD–INDI observation controller (not NMPC)');
  assert.ok(Math.abs(r.final[2]-20)<.001);
});
test('wall pacing and frame delivery cannot change plant, sensor or metric values',async()=>{
  const config={controller:'pd',scenario:'hover',seconds:.1,feedback:'eskf',seed:7};
  const plain=await R.run(ca,data,config);let waits=0;
  const paced=await R.run(ca,data,config,()=>{},()=>false,{beforeControl:async()=>{waits++;}});
  assert.equal(waits,5);assert.deepEqual(paced.final,plain.final);assert.deepEqual(paced.trace,plain.trace);
  assert.deepEqual(paced.metrics,plain.metrics);assert.deepEqual(paced.counts,plain.counts);
});
test('live targets are timestamped once and reproduce exactly as a command schedule',async()=>{
  const config={controller:'pd',scenario:'schedule',seconds:.12,preview:false,
    commands:[{t:0,speed:0,altitude:20}]};
  let tick=0,applied=[];
  const live=await R.run(ca,data,config,()=>{},()=>false,{
    beforeControl:async t=>{tick=t;},command:()=>tick===.04?{speed:3,altitude:21}:null,
    commandApplied:command=>applied.push(command)
  });
  const replay=await R.run(ca,data,live.configuration);
  assert.equal(applied.length,1);assert.deepEqual(applied[0],{t:.04,speed:3,altitude:21});
  assert.deepEqual(replay.final,live.final);assert.deepEqual(replay.trace,live.trace);
  for(const key of Object.keys(live.metrics)){
    if(typeof live.metrics[key]==='number')assert.ok(Math.abs(live.metrics[key]-replay.metrics[key])<1e-12,key);
    else assert.equal(live.metrics[key],replay.metrics[key]);
  }
  assert.equal(live.trace.find(row=>row.t===.02).reference[0],0);
  assert.equal(live.trace.find(row=>row.t===.04).reference[0],3);
  assert.notEqual(live.trace.find(row=>row.t===.04).v[0],3); // No speed snapping.
  assert.equal(Results.validateImport(live,R.validateOptions).pd.profile_id,'selected-6931');
});
test('command schedules validate ordering, bounds and comparisons include every command',()=>{
  const config={scenario:'schedule',commands:[{t:0,speed:0,altitude:20},{t:1,speed:3,altitude:20}]};
  assert.throws(()=>R.validateOptions({...config,commands:[{t:1,speed:0,altitude:20}]}),/schedule|일정/);
  assert.throws(()=>R.validateOptions({...config,commands:[...config.commands,{t:.5,speed:3,altitude:20}]}),/schedule|일정/);
  assert.throws(()=>R.validateOptions({...config,commands:[{t:0,speed:NaN,altitude:20}]}),/schedule|일정/);
  assert.deepEqual(R.referenceHorizon(0,{...config,preview:false})[20],[0,0,0,20]);
  assert.deepEqual(R.referenceHorizon(0,{...config,preview:true})[20],[3,0,0,20]);
  const base={profile_id:'selected-6931',configuration:{...R.validateOptions(config)},implementation:{input_sha256:'a'}};
  const changed=structuredClone(base);changed.configuration.commands[1].speed=4;
  assert.notEqual(Results.conditionKey(base),Results.conditionKey(changed));
});
test('wall clock waits on fast hosts and never catches up after a long suspension',async()=>{
  let wall=0;const waits=[];
  const clock=new R.ObservationClock(()=>wall,async ms=>{waits.push(ms);wall+=ms;});
  await clock.wait(.02);assert.equal(wall,20);
  wall+=60000;await clock.wait(.04);assert.equal(wall,60020);
  await clock.wait(.06);assert.equal(wall,60040);assert.deepEqual(waits,[20,20]);
  clock.reset(.06);await clock.wait(.08,()=>true);assert.equal(wall,60040);
});
