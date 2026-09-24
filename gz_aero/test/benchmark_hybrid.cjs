// Usage: node gz_aero/test/benchmark_hybrid.cjs sqprti 83 120 0
// Arguments: controller, target m/s, duration s, crosswind m/s.
// PG takes minutes. This is an explicit benchmark, not part of the fast CI suite.
const {simulator}=require('./sim_harness.cjs');
const [ctrl='sqprti',speed='83',duration='120',wind='0']=process.argv.slice(2);
if(!['sqprti','hybrid','nmpc','lqr'].includes(ctrl))throw Error('Unknown controller');
const values=[speed,duration,wind].map(Number);
if(values.some(v=>!Number.isFinite(v)||v<0)||values[1]<0.002)throw Error('Invalid benchmark arguments');
const sim=simulator(),started=performance.now();
const result=sim.run(`
  CTRL=${JSON.stringify(ctrl)};$('#spd').value=${JSON.stringify(speed)};
  $('#wsp').value=${JSON.stringify(wind)};$('#wdir').value='90';reset();
  let samples=0,limited=0,thrustError=0,thrustTracking=0,angularTracking=0;
  for(let i=0;i<${Math.round(values[1]/0.002)} && !physicsFailure;i++){
    advanceStep();
    if(T>30 && hybridStat){
      samples++;limited+=hybridStat.limited;
      thrustError+=Math.abs(hybridStat.residual[0]);
      thrustTracking+=(hybridStat.requested[0]-hybridStat.measured[0])**2;
      for(let k=1;k<4;k++)angularTracking+=(hybridStat.requested[k]-hybridStat.measured[k])**2;
    }
  }
  ({controller:CTRL,target:+$('#spd').value,wind:+$('#wsp').value,t:T,
    speed:diag().gs,altitude:X[2],failure:physicsFailure,score:score(),
    interface:samples?{windowStart:30,samples,limitedFraction:limited/samples,
      thrustAllocationMAE:thrustError/samples,thrustTrackingRMSE:Math.sqrt(thrustTracking/samples),
      angularTrackingRMSE:Math.sqrt(angularTracking/(3*samples))}:null});
`);
result.wallSeconds=(performance.now()-started)/1000;
console.log(JSON.stringify(result,null,2));
