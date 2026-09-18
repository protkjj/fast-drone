/* Derive a compact, public audit record without changing the full raw logs. */
const fs=require('node:fs'),crypto=require('node:crypto');
const path=require('node:path'),{execFileSync}=require('node:child_process');
const root=path.resolve(__dirname,'..');
const input=process.argv[2],output=process.argv[3];
if(!input||!output)throw new Error('Usage: node research/summarize_v2.cjs FULL_LOG NEW_SUMMARY');
const bytes=fs.readFileSync(input),data=JSON.parse(bytes);
if(data.reports.length!==data.cases.length*2)throw new Error('Comparison batch is incomplete');
// Summarizing an old log on a newer checkout must not relabel the experiment.
// Attach a commit only when every recorded source hash matches that Git revision.
const sourceCommit=execFileSync('git',['rev-parse',process.argv[4]||'HEAD'],{cwd:root,encoding:'utf8'}).trim();
const sourceCommitVerified=data.reports.length>0&&data.reports.every(report=>{
  const hashes=Object.entries(report.implementation?.source_sha256||{});
  return hashes.length>0&&hashes.every(([file,hash])=>{
    const relative=path.posix.normalize('research/'+file);
    if(relative.startsWith('../')||path.posix.isAbsolute(relative))return false;
    try {
      const blob=execFileSync('git',['show',`${sourceCommit}:${relative}`],{cwd:root,stdio:['ignore','pipe','ignore']});
      return crypto.createHash('sha256').update(blob).digest('hex')===hash;
    } catch {return false;}
  });
});
const reports=data.reports.map(r=>({case_id:r.case_id,configuration:r.configuration,
  implementation:r.implementation,status:r.status,failure:r.failure,simulated_seconds:r.simulated_seconds,
  wall_seconds:r.wall_seconds,counts:r.counts,event_coverage:r.event_coverage,metrics:r.metrics,final:r.final,
  failed_solves:r.solves.filter(s=>!s.success),
  timing_outliers_over_60s:r.solves.filter(s=>s.ms>60000),
  trace_checks:{first_t_s:r.trace[0].t,last_t_s:r.trace.at(-1).t,rows:r.trace.length,
    finite_states:r.trace.every(row=>row.state.every(Number.isFinite)),
    max_quaternion_norm_error:Math.max(...r.trace.map(row=>Math.abs(Math.hypot(...row.q_xyzw)-1))),
    synchronized_sensor_times:r.trace.every(row=>row.sensor.capture_t_s===row.t&&row.sensor.gyro_t_s===row.sensor.rotor_t_s)}}));
const meanSd=values=>{
  const mean=values.reduce((a,b)=>a+b,0)/values.length;
  return {n:values.length,mean,sample_sd:Math.sqrt(values.reduce((s,v)=>s+(v-mean)**2,0)/(values.length-1))};
};
const gust={};
for(const controller of ['hybrid','nmpc']){
  const group=reports.filter(r=>r.configuration.scenario==='gust'&&r.configuration.controller===controller);
  gust[controller]=Object.fromEntries(['velocity_rmse_mps','altitude_rmse_m','electrical_energy_J','outside_prop_map_fraction']
    .map(key=>[key,meanSd(group.map(r=>r.metrics[key]))]));
  gust[controller].solver_failures=group.reduce((s,r)=>s+r.metrics.solver_failures,0);
  gust[controller].solves=group.reduce((s,r)=>s+r.counts.nmpc,0);
}
const summary={schema_version:2,source_commit:sourceCommitVerified?sourceCommit:null,started_at:data.started_at,finished_at:data.updated_at,
  raw_log_sha256:crypto.createHash('sha256').update(bytes).digest('hex'),reports,gust_three_seed_summary:gust,
  timing_caveat:'Host sleep/wake and concurrent development affected wall-clock measurements. Do not use this batch for speed rankings, maximum solve latency or hard-real-time claims. Raw timings and >60 s outliers are retained.',
  limitations:['Three seeds and 3.2 seconds are exploratory, not statistical robustness evidence.',
    'The gust ends at 3 s; only 0.2 s recovery is observed, not a settling-time validation.',
    'Off-map passive continuation samples are included; no measured high-speed propulsion validation.',
    'Some solver failures remain; no cases or failed samples were excluded.',
    'Version 2 changes sensor noise stream ordering. Old and new ESKF seed traces are not identical.']};
fs.writeFileSync(output,JSON.stringify(summary,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({output,cases:reports.length,raw_log_sha256:summary.raw_log_sha256,gust},null,2));
