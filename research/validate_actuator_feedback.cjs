/* Predeclared on/off ablation: identical plant, references, costs and sensors. */
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto');
const R=require('./runtime.js');
async function main(){
  const output=process.argv[2];
  if(!output)throw new Error('Provide a NEW raw-log filename.');
  fs.writeFileSync(output,'{}',{flag:'wx'});
  const ca=await require('@casadi/casadi-wasm')();
  await ca.load_nlpsol('ipopt');await ca.load_interpolant('linear');
  const data=JSON.parse(fs.readFileSync(path.join(__dirname,'generated/selected.json')));
  const cases=[
    ...['truth','eskf'].map(feedback=>({id:'speed-'+feedback,feedback,scenario:'schedule',seconds:1.2,
      commands:[{t:0,speed:0,altitude:20},{t:.1,speed:6,altitude:20}]})),
    {id:'descent-truth',feedback:'truth',scenario:'schedule',seconds:1.2,
      commands:[{t:0,speed:0,altitude:20},{t:.1,speed:0,altitude:16}]},
    {id:'gust-eskf',feedback:'eskf',scenario:'gust',seconds:3.2,speed:3}
  ];
  const reports=[],started_at=new Date().toISOString();
  for(const item of cases)for(const enabled of [false,true]){
    const {id,...options}=item;let previous=-1;
    console.log('START '+id+'/'+(enabled?'on':'off'));
    const report=await R.run(ca,data,{...options,controller:'hybrid',preview:false,seed:42,
      hybrid_actuator_feedback:enabled},p=>{
      if(p.t-previous>=.49){previous=p.t;console.log(JSON.stringify({id,enabled,t:p.t}));}
    });
    reports.push({case_id:id,...report});
    fs.writeFileSync(output,JSON.stringify({started_at,updated_at:new Date().toISOString(),cases,reports}));
    console.log('RESULT '+JSON.stringify({id,enabled,status:report.status,metrics:report.metrics}));
  }
  const bytes=fs.readFileSync(output);
  const summary={started_at,finished_at:new Date().toISOString(),raw_log_sha256:crypto.createHash('sha256').update(bytes).digest('hex'),
    cases,reports:reports.map(r=>({case_id:r.case_id,configuration:r.configuration,implementation:r.implementation,
      status:r.status,failure:r.failure,counts:r.counts,metrics:r.metrics,final:r.final,
      envelope_active_solves:r.solves.filter(s=>s.actuator_envelope?.available).length,
      envelope_max_violation_N:Math.max(0,...r.solves.map(s=>s.envelope_violation_N||0)),
      envelope_max_roll_midpoint_error_rad_s2:Math.max(0,...r.solves.map(s=>s.actuator_envelope?.roll_secant_midpoint_error_rad_s2||0)),
      failed_solves:r.solves.filter(s=>!s.success).map(({actuator_envelope,...s})=>s)})),
    limitations:['Short, single-seed on/off Hybrid ablation, not controller superiority or robustness evidence.',
      'Nominal local 20 ms endpoint constraints are not full-horizon actuator prediction or a guaranteed nonlinear reachable set.',
      'The gust has only 0.2 s recovery; no settling-time claim. All failures and off-map samples are retained.',
      'Wall timings include concurrent development; no hard-real-time or timing ranking claim.']};
  fs.writeFileSync(output+'.summary.json',JSON.stringify(summary,null,2)+'\n',{flag:'wx'});
}
main().catch(error=>{console.error(error);process.exitCode=1;});
