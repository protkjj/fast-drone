/* Integration evidence for the common plant, not a Hybrid superiority study. */
const fs=require('node:fs'),path=require('node:path'),R=require('./runtime.js');
async function main(){
  const output=process.argv[2];if(!output)throw new Error('Provide a NEW summary filename');
  fs.writeFileSync(output,'{}',{flag:'wx'});
  const ca=await require('@casadi/casadi-wasm')();await ca.load_interpolant('linear');
  const data=JSON.parse(fs.readFileSync(path.join(__dirname,'generated/selected.json')));
  const cases=[{scenario:'hover',seconds:10,feedback:'truth'},
    {scenario:'step',seconds:10,feedback:'truth'},{scenario:'step',seconds:10,feedback:'eskf'},
    ...[7,42,2026].map(seed=>({scenario:'gust',seconds:6,feedback:'eskf',seed})),
    ...['truth','eskf'].map(feedback=>({scenario:'step',seconds:6,feedback,scales:[1.1,1.15,1.15,1.15,.9]}))];
  const evidence={started_at:new Date().toISOString(),implementation:data.provenance,cases,reports:[],
    scope:'PD–INDI observation integration only. Does not establish Hybrid superiority or physical-model accuracy.',
    timing_caveat:'Unpaced Node measurements on the development host, not browser or hard-real-time guarantees.'};
  for(const config of cases){
    const result=await R.run(ca,data,{controller:'pd',preview:false,...config});
    evidence.reports.push({configuration:result.configuration,status:result.status,failure:result.failure,
      counts:result.counts,simulated_seconds:result.simulated_seconds,wall_seconds:result.wall_seconds,
      final:result.final,metrics:result.metrics,trace_finite:result.trace.every(row=>row.state.every(Number.isFinite))});
    evidence.finished_at=new Date().toISOString();fs.writeFileSync(output,JSON.stringify(evidence,null,2)+'\n');
    console.log(JSON.stringify(evidence.reports.at(-1)));
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;});
