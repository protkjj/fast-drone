/* Paired, predeclared tests. No per-controller tuning or failed-case exclusion. */
const fs=require('node:fs');
const path=require('node:path');
const R=require('./runtime.js');
async function main(){
  const output=process.argv[2];
  if(!output)throw new Error('Provide a NEW output filename; old experiments are preserved.');
  fs.writeFileSync(output,JSON.stringify({reports:[]}),{flag:'wx'});
  const ca=await require('@casadi/casadi-wasm')();
  await ca.load_nlpsol('ipopt');await ca.load_interpolant('linear');
  const data=JSON.parse(fs.readFileSync(path.join(__dirname,'generated/selected.json')));
  const cases=[
    {id:'nominal-truth',feedback:'truth',scales:[1,1,1,1,1],seed:42,scenario:'step',seconds:2},
    {id:'nominal-eskf',feedback:'eskf',scales:[1,1,1,1,1],seed:42,scenario:'step',seconds:2},
    {id:'combined-truth',feedback:'truth',scales:[1.1,1.15,1.15,1.15,.9],seed:42,scenario:'step',seconds:2},
    ...[7,42,2026].map(seed=>({id:'gust-eskf-'+seed,feedback:'eskf',scales:[1,1,1,1,1],seed,scenario:'gust',seconds:3.2}))
  ];
  const reports=[],started_at=new Date().toISOString();
  for(const item of cases)for(const controller of ['hybrid','nmpc']){
    const {id,...config}=item;
    console.log('START '+id+'/'+controller);
    let last=-1;
    const r=await R.run(ca,data,{...config,controller,speed:3,preview:true},p=>{
      if(p.t-last>=.49){last=p.t;console.log(JSON.stringify({case:id,controller,t:p.t,solves:p.solves}));}
    });
    reports.push({case_id:id,...r});
    fs.writeFileSync(output,JSON.stringify({started_at,updated_at:new Date().toISOString(),cases,reports}));
    console.log('RESULT '+JSON.stringify({case:id,controller,status:r.status,metrics:r.metrics}));
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;});
