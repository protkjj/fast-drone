const {test}=require('node:test'),assert=require('node:assert/strict');
const R=require('../results.js'),runtime=require('../runtime.js');
function report(controller='hybrid'){
  const state=[0,0,20,0,0,0,0,0,0,1,0,0,0,100,100,100,100,1];
  return {schema_version:2,configuration:runtime.validateOptions({scenario:'hover',seconds:.2,controller}),
    profile_id:'selected-6931',implementation:{input_sha256:'a'.repeat(64)},status:'completed',
    simulated_seconds:.2,metrics:{velocity_rmse_mps:0},counts:{plant:200},final:state.slice(),
    trace:[0,.2].map(t=>({t,state:state.slice(),reference:[0,0,0,20],command_rad_s:[100,100,100,100],rotor_rad_s:[100,100,100,100]}))};
}
test('import verifies versions, finite data, time order and paired conditions',()=>{
  assert.equal(Object.keys(R.validateImport({results:{hybrid:report(),nmpc:report('nmpc')}},runtime.validateOptions)).length,2);
  const different=report('nmpc');different.configuration.seed++;
  assert.throws(()=>R.validateImport({results:{hybrid:report(),nmpc:different}},runtime.validateOptions),/조건/);
  const invalid=report();invalid.trace[1].t=0;
  assert.throws(()=>R.validateImport(invalid,runtime.validateOptions),/시각/);
  assert.throws(()=>R.validateImport({schema_version:1},runtime.validateOptions),/v2/);
  const nan=report();nan.trace[1].state[0]=NaN;
  assert.throws(()=>R.validateImport(nan,runtime.validateOptions),/상태/);
});
test('replay follows recorded simulation time and interpolates quaternion short arc without changing logs',()=>{
  const r=report();r.trace[1].state[0]=2;r.trace[1].state[9]=-1;
  const before=JSON.stringify(r),half=R.sample(r.trace,.1);
  assert.equal(half.state[0],1);assert.equal(half.state[9],1);
  assert.equal(R.sample(r.trace,10).t,.2);assert.equal(R.sample(r.trace,-1).t,0);
  assert.equal(JSON.stringify(r),before);
});
test('historical logs are not relabeled as actuator-aware Hybrid',()=>{
  const old=report();delete old.configuration.hybrid_actuator_feedback;
  assert.equal(R.validateImport(old,runtime.validateOptions).hybrid.configuration.hybrid_actuator_feedback,false);
  const different=report('nmpc');different.configuration.hybrid_actuator_feedback=true;
  assert.throws(()=>R.validateImport({results:{hybrid:report(),nmpc:different}},runtime.validateOptions),/조건/);
});
