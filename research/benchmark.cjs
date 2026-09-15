/* Sequential comparisons: identical inputs, no per-controller retuning. */
const fs=require('node:fs');
const path=require('node:path');
const {run}=require('./runtime.js');
async function main() {
  const ca=await require('@casadi/casadi-wasm')();
  await ca.load_nlpsol('ipopt');await ca.load_interpolant('linear');
  const data=JSON.parse(fs.readFileSync(path.join(__dirname,'generated/selected.json'),'utf8'));
  const output=process.argv[2]||path.join(__dirname,'generated/benchmark.json');
  const reports=[];
  for(const configuration of [
    {feedback:'truth',scales:[1,1,1,1,1]},
    {feedback:'eskf',scales:[1,1,1,1,1]},
    {feedback:'truth',scales:[1.1,1.15,1.15,1.15,.9]}
  ]) for(const controller of ['hybrid','nmpc']) {
    console.log(`START ${controller}/${configuration.feedback}/${configuration.scales.join(',')}`);
    const result=await run(ca,data,{...configuration,controller,seconds:2,scenario:'step',speed:3,seed:42},
      progress=>console.log(JSON.stringify({progress_s:progress.t,controller,feedback:configuration.feedback,
        solves:progress.solves,vx:progress.v[0]})));
    reports.push(result);
    fs.writeFileSync(output,JSON.stringify({reports},null,2)+'\n');
    console.log(JSON.stringify({controller,feedback:configuration.feedback,scales:configuration.scales,
      failure:result.failure,final_velocity:result.final.slice(3,6),metrics:result.metrics}));
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;});
