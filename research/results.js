/* Pure log validation and display interpolation; never used by the plant. */
(function(root){
  'use strict';
  const vector=(value,n)=>Array.isArray(value)&&value.length===n&&value.every(Number.isFinite);
  function conditionKey(report){
    const c=report.configuration;
    return JSON.stringify([report.profile_id,report.implementation?.input_sha256,
      c.feedback,c.scenario,c.seconds,c.speed,c.altitude,c.seed,c.preview,c.scales,c.commands||null]);
  }
  function validateImport(value,validateOptions){
    const reports=value?.results?Object.values(value.results):[value];
    if(!reports.length||reports.length>2)throw new Error('한 번에 같은 조건의 결과 1–2개만 가져올 수 있습니다.');
    const result={};
    for(const r of reports){
      if(r?.schema_version!==2)throw new Error('시각·단위가 명확한 v2 로그만 재생할 수 있습니다. 이전 로그는 원본 파일로 보존하세요.');
      const cfg=validateOptions(r.configuration);
      if(result[cfg.controller])throw new Error('같은 제어기의 중복 결과입니다.');
      if(!['selected-6931','simple-passive'].includes(r.profile_id))throw new Error('알 수 없는 기체 프로필입니다.');
      if(!/^[a-f0-9]{64}$/.test(r.implementation?.input_sha256||''))throw new Error('모델·구현 해시가 없는 로그입니다.');
      if(!['completed','stopped','failed'].includes(r.status)||!Number.isFinite(r.simulated_seconds)||r.simulated_seconds<0||r.simulated_seconds>120)throw new Error('유효하지 않은 실행 상태입니다.');
      if(!vector(r.final,18)||!Array.isArray(r.trace)||!r.trace.length||r.trace.length>120001)throw new Error('상태 또는 기록 길이가 잘못되었습니다.');
      if(!r.metrics||!r.counts||Object.values(r.metrics).some(v=>v!==null&&!Number.isFinite(v))||Object.values(r.counts).some(v=>!Number.isFinite(v)||v<0))throw new Error('유효하지 않은 결과 지표입니다.');
      let previous=-1;
      for(const row of r.trace){
        if(!Number.isFinite(row.t)||row.t<0||row.t<=previous||row.t>r.simulated_seconds+1e-8||
          !vector(row.state,18)||!vector(row.reference,4)||Math.abs(Math.hypot(...row.state.slice(6,10))-1)>.01)
          throw new Error('시각 순서·상태·자세가 유효하지 않은 로그입니다.');
        if(!vector(row.command_rad_s,4)||!vector(row.rotor_rad_s,4))throw new Error('rad/s 회전수 채널이 필요합니다.');
        previous=row.t;
      }
      if(r.trace[0].t!==0||Math.abs(previous-r.simulated_seconds)>1e-8)throw new Error('초기·종료 기록이 누락되었습니다.');
      // Derive display values from the canonical state, not redundant aliases.
      result[cfg.controller]={...r,configuration:cfg,trace:r.trace.map(row=>({...row,
        v:row.state.slice(3,6),z:row.state[2],position_m:row.state.slice(0,3),q_xyzw:row.state.slice(6,10)}))};
    }
    if(new Set(Object.values(result).map(conditionKey)).size>1)throw new Error('기체·구현 버전·시험 조건이 달라 직접 비교할 수 없습니다. 각 로그를 따로 가져오세요.');
    return result;
  }
  function sample(trace,t){
    if(!trace?.length)return null;
    if(t<=trace[0].t)return {t:trace[0].t,state:trace[0].state.slice()};
    if(t>=trace.at(-1).t)return {t:trace.at(-1).t,state:trace.at(-1).state.slice()};
    let lo=0,hi=trace.length-1;
    while(hi-lo>1){const mid=(lo+hi)>>1;if(trace[mid].t<=t)lo=mid;else hi=mid;}
    const a=trace[lo],b=trace[hi],fraction=(t-a.t)/(b.t-a.t);
    const state=a.state.map((v,i)=>v+fraction*(b.state[i]-v));
    const qa=a.state.slice(6,10),qb=b.state.slice(6,10);
    const sign=qa.reduce((s,v,i)=>s+v*qb[i],0)<0?-1:1;
    const q=qa.map((v,i)=>v+fraction*(sign*qb[i]-v)),length=Math.hypot(...q);
    q.forEach((v,i)=>state[6+i]=v/length);
    return {t,state};
  }
  const api={validateImport,conditionKey,sample};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.ResearchResults=api;
})(globalThis);
