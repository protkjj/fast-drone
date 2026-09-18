'use strict';
const $=id=>document.getElementById(id);
const SELECTED_DISPLAY_LABEL='선정안 CSV 기준 · STL 파생 표시';
const results={};
let worker=null,vehicle=null,running=false,drawModel=()=>{},updatePath=()=>{},appendLivePath=()=>{};
let playing=false,replayTime=0,imported=false,customScales=null,mode='compare',paused=false,recordedCommands=null;
let replayActive=false,resetRequested=false,lastPlotWall=0;
// Retain the ablation setting when an imported experiment is reused. No layout
// or flight-control widgets change. Mixed flight results keep this opt-in.
let hybridActuatorFeedback=new URL(location.href).searchParams.get('actuator_feedback')==='1';
const liveTraces={};
const controllerNames={pd:'PD–INDI · 조작용 baseline',hybrid:'Hybrid',nmpc:'NMPC 단독'};
const settingIds=['profile','controller','feedback','scenario','seconds','speed','altitude','preview','mismatch','seed','log-hz','wind-speed','wind-angle'];
const modeSettings={compare:null,observe:{profile:'selected',controller:'both',feedback:'truth',scenario:'hover',seconds:'30',speed:'0',altitude:'20',preview:'false',mismatch:'nominal',seed:'42','log-hz':'50'}};
const pdOption=document.createElement('option');pdOption.value='pd';pdOption.textContent=controllerNames.pd;$('replay-controller').append(pdOption);
const compareRecord=document.createElement('button');compareRecord.id='compare-record';compareRecord.textContent='이 목표 일정으로 정밀 비교';compareRecord.disabled=true;
$('reuse').parentElement.append(compareRecord);
function setMode(next,restore=true){
  if(running)return;
  if(restore)modeSettings[mode]=Object.fromEntries(settingIds.map(id=>[id,$(id).value]));
  mode=next;setPlaying(false);
  if(restore&&modeSettings[mode])for(const [id,value] of Object.entries(modeSettings[mode]))$(id).value=value;
  for(const kind of ['observe','compare']){
    $('mode-'+kind).setAttribute('aria-pressed',String(kind===mode));
    document.querySelectorAll('.'+kind+'-only').forEach(el=>el.hidden=kind!==mode);
  }
  $('run').textContent=mode==='observe'?'새 비행':'정밀 계산 시작';$('pause').hidden=mode!=='observe';
  $('settings').open=!matchMedia('(max-width:900px)').matches;
  updateModeNote();
  $('status').textContent=mode==='observe'?'비행 시작 → 슬라이더로 속도·고도·바람 조작':'정밀 비교 준비됨. 짧은 호버로 확인 후 속도·외란 시험을 선택하세요.';
  $('record-panel').open=mode==='compare';
  $('progress').value=0;scenarioNote(false);updateAircraftLabel();
  updateFlightActions();
  const url=new URL(location.href);url.searchParams.set('mode',mode);history.replaceState(null,'',url);
}
function updateModeNote(){
  $('mode-note').textContent=mode!=='observe'?'Hybrid와 NMPC 단독을 같은 기체·조건에서 계산합니다.':
    $('observe-controller').value==='pd'?'PD–INDI baseline · 빠른 조작용이며 Hybrid가 아닙니다. 목표와 바람은 비행 중 자동 반영됩니다.':
    `${controllerNames[$('observe-controller').value]} · 실제 IPOPT 계산으로 비행합니다. 풀이가 느리면 비행 진행도 느려지며, 다른 제어기로 대체하지 않습니다.`;
  if(hybridActuatorFeedback)$('mode-note').textContent+=' · 실험용 Hybrid 모터 능력 제약 ON';
}
$('observe-controller').addEventListener('change',updateModeNote);
function updateAircraftLabel(){
  document.querySelector('.aircraft-tag').textContent=$('profile').value==='selected'?
    '선정안 6931 · 1.712 kg · CSV 기준 형상 · 개념설계 모델':'단순 검증 기체 · 2 kg · 선정 기체가 아닌 구조 검증용 모델';
}
$('mode-observe').addEventListener('click',()=>setMode('observe'));
$('mode-compare').addEventListener('click',()=>setMode('compare'));
const mismatches={nominal:[1,1,1,1,1],mass:[1.1,1,1,1,1],inertia:[1,1.15,1.15,1.15,1],
  prop:[1,1,1,1,.9],combined:[1.1,1.15,1.15,1.15,.9]};
if(matchMedia('(max-width:850px)').matches)$('settings').open=false;
let rotorRates=[0,0,0,0],rotorFrame=null,lastRotorFrame=0;
function animateModel(now){
  rotorFrame=null;
  if((!running||paused)&&!playing)return;
  if(now-lastRotorFrame>=1000/30){
    const dt=Math.min(.1,(now-lastRotorFrame)/1000);lastRotorFrame=now;
    if(playing){
      replayTime=Math.min(replayResult().simulated_seconds,replayTime+dt*Number($('replay-speed').value));
      renderReplay();if(replayTime>=replayResult().simulated_seconds)setPlaying(false);
    }
    if(vehicle&&vehicle.visible){ResearchSTL.animateRotors(vehicle,rotorRates,dt,(running&&!paused)||playing);drawModel();}
  }
  if(running||playing)rotorFrame=requestAnimationFrame(animateModel);
}
function startFrames(){if(rotorFrame===null){lastRotorFrame=performance.now();rotorFrame=requestAnimationFrame(animateModel);}}
function setPlaying(value){
  playing=value;$('play').textContent=value?'일시정지':'재생';
  updateFlightActions();
  if(value)startFrames();else if(vehicle){ResearchSTL.animateRotors(vehicle,rotorRates,0,false);drawModel();}
}
const colors={pd:'#f2aa4c',hybrid:'#59c8f5',nmpc:'#db9bff'};
function setBusy(value) {
  running=value; $('run').disabled=value; $('stop').disabled=!value;
  for(const id of ['mode-observe','mode-compare'])$(id).disabled=value;
  $('pause').disabled=!value||mode!=='observe';$('observe-controller').disabled=value;
  $('compare-record').disabled=value||!replayResult();
  document.querySelectorAll('.controls input,.controls select').forEach(el=>el.disabled=value);
  $('import').disabled=value;$('reuse').disabled=value||!replayResult();
  $('play').disabled=value||!replayResult();$('timeline').disabled=value||!replayResult();
  $('replay-controller').disabled=value||!replayResult();
  if(value){paused=false;$('pause').textContent='일시정지';setPlaying(false);if(vehicle)vehicle.visible=$('profile').value==='selected';startFrames();}
  if(!value&&vehicle){ResearchSTL.animateRotors(vehicle,rotorRates,0,false);drawModel();}
  updateFlightActions();
}
function updateFlightActions(){
  const busy=running&&($('pause').disabled||mode!=='observe');
  const label=running?(paused?'▶ 계속':busy?'처리 중…':'❚❚ 일시정지'):
    playing?'❚❚ 재생 정지':replayActive&&replayResult()?'▶ 기록 재생':'▶ 시작';
  $('go').textContent=label+' (Space)';$('quick-go').textContent=label;
  $('go').disabled=busy;$('quick-go').disabled=busy;
}
function toggleFlight(){
  if(running){if(mode==='observe'&&!$('pause').disabled)$('pause').click();return;}
  if(replayActive&&replayResult())$('play').click();else $('run').click();
}
$('go').addEventListener('click',toggleFlight);$('quick-go').addEventListener('click',toggleFlight);
function resetView(){
  resetRequested=false;setPlaying(false);replayActive=false;replayTime=0;
  const altitude=Number($('altitude').value),z=Number.isFinite(altitude)?altitude:20;
  rotorRates=[0,0,0,0];updatePath(null);
  if(vehicle){vehicle.visible=$('profile').value==='selected';vehicle.position.set(0,0,z);vehicle.quaternion.set(0,-Math.SQRT1_2,0,Math.SQRT1_2);
    for(const rotor of vehicle.userData.rotors)rotor.rotation.x=0;drawModel();}
  updateTelemetry({t:0,p:[0,0,z],v:[0,0,0],z,soc:1});
  $('timeline').value='0';$('timeline-time').textContent='0.00 s';$('progress').value=0;
  $('status').textContent='초기 상태 · Space로 시작. 이전 계산 기록은 저장·재생할 수 있습니다.';
  $('live-limits').textContent='새 비행 준비 · 기본 검증은 저속 영역입니다.';$('live-limits').classList.toggle('bad',false);
  $('model-caption').textContent=SELECTED_DISPLAY_LABEL+' · 초기 상태 · 선정 기체 공통 물리 사용';
  for(const id of ['velocity-chart','altitude-chart','angle-chart'])$(id).replaceChildren();
  updateFlightActions();
}
$('reset').addEventListener('click',()=>{
  if(running){resetRequested=true;$('stop').click();$('status').textContent='초기화 요청 · 현재 풀이 후 기록을 보존하고 초기 상태로 돌아갑니다.';}
  else resetView();
});
document.addEventListener('keydown',event=>{
  const target=event.target,tag=target?.tagName;
  if(event.defaultPrevented||event.ctrlKey||event.metaKey||event.altKey||event.repeat||event.isComposing||target?.isContentEditable)return;
  // A range does not use Space, so pausing still works after dragging it.
  // Text/numeric fields, selects and native button activation retain their keys.
  if(['TEXTAREA','SELECT','BUTTON','SUMMARY'].includes(tag)||tag==='INPUT'&&target.type!=='range')return;
  if(event.code==='Space'){event.preventDefault();toggleFlight();}
  else if(tag!=='INPUT'&&event.key.toLowerCase()==='r'){event.preventDefault();$('reset').click();}
  else if(tag!=='INPUT'&&event.key.toLowerCase()==='f'){event.preventDefault();$('follow').click();}
});
for(const [id,property,direction] of [['grip-left','--colL',1],['grip-right','--colR',-1]]){
  const grip=$(id);let drag=null;
  const setWidth=value=>{
    const width=Math.max(190,Math.min(innerWidth*.3,value));
    $('sim-shell').style.setProperty(property,width+'px');grip.setAttribute('aria-valuenow',String(Math.round(width)));
  };
  grip.addEventListener('pointerdown',event=>{
    const panel=id==='grip-left'?document.querySelector('.left'):document.querySelector('.right');
    drag={x:event.clientX,width:panel.getBoundingClientRect().width};grip.setPointerCapture(event.pointerId);event.preventDefault();
  });
  grip.addEventListener('pointermove',event=>{if(drag)setWidth(drag.width+direction*(event.clientX-drag.x));});
  grip.addEventListener('pointerup',()=>drag=null);grip.addEventListener('pointercancel',()=>drag=null);
  grip.addEventListener('keydown',event=>{
    if(!['ArrowLeft','ArrowRight','Home','End'].includes(event.key))return;
    event.preventDefault();const width=(id==='grip-left'?document.querySelector('.left'):document.querySelector('.right')).getBoundingClientRect().width;
    setWidth(event.key==='Home'?190:event.key==='End'?innerWidth*.3:width+direction*(event.key==='ArrowRight'?10:-10));
  });
}
const number=value=>!Number.isFinite(value)?'—':value!==0&&Math.abs(value)<.001?value.toExponential(2):value.toLocaleString('ko-KR',{maximumFractionDigits:3});
function showResults() {
  const kinds=results.pd?Object.keys(results):['hybrid','nmpc'];
  const heading=$('metrics').parentElement.querySelector('thead tr');heading.replaceChildren();
  for(const label of ['지표',...kinds.map(k=>controllerNames[k])]){const th=document.createElement('th');th.textContent=label;heading.append(th);}
  const rows=[['속도 RMSE · m/s','velocity_rmse_mps'],['고도 RMSE · m','altitude_rmse_m'],
    ['추정 위치 RMSE · m','position_estimation_rmse_m'],['소비 전기에너지 · J','electrical_energy_J'],
    ['명령 포화 비율 · %','saturation_fraction',100],['추진 맵 밖 비율 · %','outside_prop_map_fraction',100],
    ['전류 제한 작동 비율 · %','current_limited_fraction',100],['전압 제한 작동 비율 · %','voltage_limited_fraction',100],
    ['모터 추종 제한 비율 · %','tracking_limited_fraction',100],['제동 불가·자연 감속 비율 · %','coasting_fraction',100],
    ['명목 할당 잔차 발생 비율 · %','allocation_limited_fraction',100],
    ['총추력 할당 잔차 최대 · N','allocation_thrust_residual_max_N'],
    ['각가속도 할당 잔차 최대 · rad/s²','allocation_alpha_residual_max_rad_s2'],
    ['풀이 실패 횟수','solver_failures'],['풀이 p50 · ms','solve_ms_p50'],['풀이 p95 · ms','solve_ms_p95'],
    ['풀이 최대 · ms (첫 풀이 포함)','solve_ms_max'],['20 ms 초과 횟수','deadline_misses_20ms'],
    ['모터 에너지 잔차 최대 · W','motor_energy_residual_W'],['버스 전압 잔차 최대 · V','bus_residual_V']];
  const body=$('metrics'); body.replaceChildren();
  function addRow(label,values) {
    const tr=document.createElement('tr'), title=document.createElement('td'); title.textContent=label;tr.append(title);
    values.forEach(text=>{const td=document.createElement('td');td.textContent=text;tr.append(td);});body.append(tr);
  }
  addRow('실행 결과',kinds.map(k=>results[k]?(results[k].failure||(results[k].status==='stopped'?'사용자 중지 · 부분 결과':null)||
    (results[k].metrics.solver_failures?'풀이 실패 포함 · 비교 해석 주의':'계산 완료')):'—'));
  addRow('시뮬레이션 시간 · s',kinds.map(k=>number(results[k]?.simulated_seconds)));
  addRow('실제 경과 시간 · s (대기 포함)',kinds.map(k=>number(results[k]?.wall_seconds)));
  for(const [label,key,scale=1] of rows) addRow(label,kinds.map(k=>{
    const value=results[k]?.metrics[key];return number(value==null?null:value*scale);
  }));
  addRow('속도 입력 / 외란 종료 포함',kinds.map(k=>{
    const r=results[k];if(!r)return '—';const e=r.event_coverage;
    return (r.configuration.scenario==='schedule'?`목표 일정 ${r.configuration.commands.length}개`:r.configuration.scenario==='hover'?'해당 없음':e?.speed_step?'입력 포함':'입력 미도달')+
      (r.configuration.scenario==='gust'?(e?.gust_completed?' / 외란 종료 포함':' / 외란 미완료'):'');
  }));
  addRow('NMPC / INDI / IMU 횟수',kinds.map(k=>{const c=results[k]?.counts;return c?`${c.nmpc} / ${c.indi} / ${c.imu}`:'—';}));
  addRow('GPS 융합 / 재적분 IMU 횟수',kinds.map(k=>{const c=results[k]?.counts;return c?`${c.gps_fused} / ${c.gps_replayed_imu}`:'—';}));
  drawCharts();
  $('download').disabled=Object.keys(results).length===0;
  const result=Object.values(results)[0];
  $('result-context').textContent=result?`${imported?'가져온 기록 · ':'현재 실행 · '}${result.profile_id} · ${result.configuration.scenario} · seed ${result.configuration.seed} · ${result.configuration.preview?'미래 목표 예고':'비예고'} · 구현 ${result.implementation?.input_sha256?.slice(0,12)||'알 수 없음'}\n제한 비율은 각 tick에 4개 모터 중 하나라도 해당 제한이 작동한 비율입니다. 모터 추종 제한은 전류·전압·회생 제동 불가를 포함하며 속도 오차 자체와는 다릅니다.`:'';
  if(Object.values(results).some(r=>r.status!=='completed'))
    $('result-context').textContent+='\n부분·실패 결과가 포함되어 있습니다. 관측 시간이 다른 RMSE와 에너지를 직접 순위 비교하지 마세요.';
  if(results.pd)$('result-context').textContent+='\nPD–INDI 관찰 기록입니다. 이 결과는 Hybrid 또는 NMPC 성능이 아닙니다.';
}
function drawChart(id,value,reference) {
  const svg=$(id), ns='http://www.w3.org/2000/svg';svg.replaceChildren();
  // Use actual CSS pixels, not a large fixed viewBox that shrinks tick labels.
  const width=Math.max(100,svg.clientWidth||620),height=svg.clientHeight||135;
  svg.setAttribute('viewBox',`0 0 ${width} ${height}`);
  const left=width<180?36:46,right=width-8,bottom=height-23,top=12;
  const element=(name,attributes,text)=>{const node=document.createElementNS(ns,name);Object.entries(attributes).forEach(([k,v])=>node.setAttribute(k,v));if(text!==undefined)node.textContent=text;svg.append(node);return node;};
  const sources=running?{...results,...Object.fromEntries(Object.entries(liveTraces).map(([k,trace])=>[k,{trace}]))}:results;
  const all=Object.values(sources).flatMap(r=>r.trace);if(!all.length)return;
  let lo=Infinity,hi=-Infinity,time=.001;
  for(const point of all){for(const v of [value(point),reference?.(point)])if(Number.isFinite(v)){lo=Math.min(lo,v);hi=Math.max(hi,v);}time=Math.max(time,point.t);}
  if(!Number.isFinite(lo)){
    element('text',{x:width/2,y:height/2,fill:'#aebac6','font-size':11,'text-anchor':'middle'},'저속 · 방향 미정');
    return;
  }
  const margin=Math.max(.05,(hi-lo)*.12);lo-=margin;hi+=margin;
  const sx=t=>left+t/time*(right-left),sy=v=>bottom-(v-lo)/(hi-lo)*(bottom-top);
  const ticks=width<240?2:4,tick=v=>Math.abs(v)>0&&Math.abs(v)<.01?v.toExponential(0):String(Number(v.toFixed(Math.abs(v)>=100?0:2)));
  for(let i=0;i<=ticks;i++) {
    const v=lo+(hi-lo)*i/ticks,y=sy(v);
    element('line',{x1:left,x2:right,y1:y,y2:y,stroke:'#30404e'});
    element('text',{x:left-5,y:y+4,fill:'#aebac6','font-size':10,'text-anchor':'end'},tick(v));
    element('text',{x:sx(time*i/ticks),y:height-7,fill:'#aebac6','font-size':10,'text-anchor':i===ticks?'end':'middle'},tick(time*i/ticks));
  }
  // Keep long 1 kHz logs responsive. The exported log retains every sample.
  const path=(trace,fn)=>{let pen=false;return trace.filter((_,i)=>i%Math.max(1,Math.ceil(trace.length/2000))===0||i===trace.length-1)
    .map(point=>{const v=fn(point);if(!Number.isFinite(v)){pen=false;return '';}
      const segment=`${pen?'L':'M'}${sx(point.t).toFixed(2)},${sy(v).toFixed(2)}`;pen=true;return segment;}).join(' ');};
  if(reference)element('path',{d:path(Object.values(sources)[0].trace,reference),fill:'none',stroke:'#aebac6','stroke-dasharray':'5 4','stroke-width':1.5});
  for(const [kind,result] of Object.entries(sources)) element('path',{d:path(result.trace,value),fill:'none',stroke:colors[kind],'stroke-width':2});
}
function drawCharts(){
  drawChart('velocity-chart',p=>p.v[0],p=>p.reference?.[0]??0);
  drawChart('altitude-chart',p=>p.z,p=>p.reference?.[3]??Number($('altitude').value));
  drawChart('angle-chart',airAngle);
}
function airAngle(point){
  const q=point.q||point.q_xyzw||point.state?.slice(6,10);if(!q)return null;
  const relative=point.v.map((v,i)=>v-(point.environment?.[i]||0)),speed=Math.hypot(...relative);
  if(speed<.5)return null; // Direction of near-zero airspeed is undefined.
  const axis=ResearchRuntime.rotate(q,[1,0,0]);
  return Math.acos(Math.max(-1,Math.min(1,axis.reduce((s,v,i)=>s+v*relative[i],0)/speed)))*180/Math.PI;
}
function plotLiveFrame(frame){
  const trace=liveTraces[frame.controller]||=([]);
  if(trace.at(-1)?.t===frame.t)trace.pop();
  trace.push({...frame});if(trace.length>6001)trace.shift();
  if(performance.now()-lastPlotWall<180)return;lastPlotWall=performance.now();
  drawCharts();
}
if(typeof ResizeObserver!=='undefined')new ResizeObserver(()=>{
  if(running||replayActive)drawCharts();
}).observe($('velocity-chart'));
$('run').addEventListener('click',()=>{
  clearTimeout(liveTargetTimer);liveTargetTimer=null;
  const seconds=Number($('seconds').value), speed=Number($('speed').value),seed=Number($('seed').value);
  const mismatch=mismatches[$('mismatch').value]||customScales;
  let options;
  const altitude=Number($('altitude').value),scenario=mode==='observe'?'schedule':$('scenario').value;
  const commands=mode==='observe'&&$('scenario').value!=='schedule'?[{t:0,...readLiveTarget()}]:recordedCommands?.filter(command=>command.t<=seconds);
  try{options=ResearchRuntime.validateOptions({seconds,speed,seed,controller:mode==='observe'?$('observe-controller').value:'hybrid',feedback:$('feedback').value,scenario,commands,
    wind_speed:mode==='observe'?Number($('live-wind-speed').value):Number($('wind-speed').value),
    wind_angle:mode==='observe'?Number($('live-wind-angle').value):Number($('wind-angle').value),
    scales:mismatch,altitude,preview:mode==='observe'?false:$('preview').value==='true',
    hybrid_actuator_feedback:hybridActuatorFeedback,log_hz:Number($('log-hz').value)});}
  catch(error){$('errors').textContent=error.message;$('settings').open=true;return;}
  setPlaying(false);imported=false;
  replayActive=false;resetRequested=false;Object.keys(liveTraces).forEach(k=>delete liveTraces[k]);lastPlotWall=0;
  Object.keys(results).forEach(k=>delete results[k]);showResults();$('errors').textContent='';
  updatePath(null);
  $('target-state').textContent=`초기 목표: ${number(commands?.[0]?.speed??speed)} m/s · ${number(commands?.[0]?.altitude??altitude)} m`;
  $('model-caption').textContent=`${mode==='observe'?controllerNames[options.controller]:'정밀 비교'} · 선정 기체 공통 물리 · 위치·자세는 적분 상태`;
  setBusy(true);$('progress').value=0;
  if(!worker) {
    worker=new Worker('./worker.js');
    worker.onerror=event=>{$('errors').textContent=event.message;setBusy(false);};
    worker.onmessage=event=>{
      const m=event.data;
      if(m.type==='status') $('status').textContent=m.text;
      if(m.type==='progress') {
        if(m.t===0&&m.solves===0)updatePath(null);
        if(m.rpm)rotorRates=m.rpm.slice();
        $('status').textContent=`${controllerNames[m.controller]} · ${number(m.t)} / ${number(m.total)} s\n${m.controller==='pd'?'관찰 진행 · 1 ms 적분 유지':`IPOPT ${m.solves}회 · 풀이 중 시뮬레이션 대기`}`;
        updateTelemetry(m);
        plotLiveFrame(m);
        $('progress').value=m.t/m.total;
        if(vehicle&&m.q) {vehicle.quaternion.set(...m.q);if(m.p){vehicle.position.set(...m.p);appendLivePath(m.p);}drawModel();}
      }
      if(m.type==='result') {
        results[m.result.configuration.controller]=m.result;showResults();
        $('replay-controller').value=m.result.configuration.controller;setupReplay();
        rotorRates=m.result.final.slice(13,17);
        if(vehicle){vehicle.position.set(...m.result.final.slice(0,3));vehicle.quaternion.set(...m.result.final.slice(6,10));drawModel();}
      }
      if(m.type==='done') {
        paused=false;setBusy(false);setupReplay(true);$('status').textContent=m.stopped?(m.count?'중지됨. 현재까지 계산된 부분 결과도 보존했습니다.':'모델 준비 중에 중지했습니다. 아직 계산된 결과는 없습니다.'):'계산 완료. 기록을 재생하거나 같은 목표 일정으로 정밀 비교할 수 있습니다.';
        if($('target-state').textContent.startsWith('적용 대기'))$('target-state').textContent='실행이 끝나 대기 중 변경은 적용되지 않았습니다. 실제 적용된 일정은 저장한 로그에 남습니다.';
        showResults();updateFlightActions();if(resetRequested)resetView();
      }
      if(m.type==='target-applied')$('target-state').textContent=`${number(m.command.t)} s 적용 · 목표 ${number(m.command.speed)} m/s / ${number(m.command.altitude)} m · 바람 ${number(m.command.wind_speed)} m/s / ${number(m.command.wind_angle)}° · 기록됨`;
      if(m.type==='command-error')$('errors').textContent=m.text;
      if(m.type==='paused'){
        paused=m.paused;$('pause').disabled=false;$('pause').textContent=paused?'계속 관찰':'일시정지';
        if(paused){$('status').textContent=`${number(m.t)} s에서 일시정지 · 물리 시간도 멈춤`;if(vehicle){ResearchSTL.animateRotors(vehicle,rotorRates,0,false);drawModel();}}
        else startFrames();
        updateFlightActions();
      }
      if(m.type==='error') {$('errors').textContent=m.text;setBusy(false);}
    };
  }
  worker.postMessage({type:'run',mode,profile:$('profile').value,
    controllers:$('controller').value==='both'?['hybrid','nmpc']:[$('controller').value],
    options});
});
$('stop').addEventListener('click',()=>{worker?.postMessage({type:'stop'});$('status').textContent='중지 요청됨. 현재 풀이·적분 구간이 끝나면 기록을 보존합니다.';$('stop').disabled=true;});
$('pause').addEventListener('click',()=>{if(running&&mode==='observe'){
  worker.postMessage({type:'pause',paused:!paused});$('pause').disabled=true;
  $('pause').textContent=paused?'재개 요청 중…':'일시정지 요청 중…';
  updateFlightActions();
}});
function applyLiveTarget(hover=false){
  clearTimeout(liveTargetTimer);liveTargetTimer=null;
  if(mode!=='observe')return;
  if(hover)$('live-target-speed').value='0';
  const command=readLiveTarget();
  try{ResearchRuntime.validateOptions({scenario:'schedule',commands:[{t:0,...command}]});}
  catch(error){$('errors').textContent=error.message;return;}
  syncLiveControls();$('errors').textContent='';
  if(!running){
    // A deliberate new initial target cancels an imported future schedule.
    $('scenario').value='hover';
    $('target-state').textContent='시작 목표를 설정했습니다. 비행 시작을 누르면 적용됩니다.';return;
  }
  worker.postMessage({type:'target',...command});
  $('target-state').textContent='적용 대기 · 현재 풀이 후 다음 20 ms 제어 시각에 반영됩니다.';
}
const livePairs=[['live-target-speed','speed-slider'],['live-target-altitude','altitude-slider'],['live-wind-speed','wind-speed-slider'],['live-wind-angle','wind-angle-slider']];
let liveTargetTimer=null;
function readLiveTarget(){
  const read=id=>$(id).value.trim()===''?NaN:Number($(id).value);
  return {speed:read('live-target-speed'),altitude:read('live-target-altitude'),
    wind_speed:read('live-wind-speed'),wind_angle:read('live-wind-angle')};
}
function syncLiveControls(){
  const altitude=Number($('live-target-altitude').value);
  $('altitude-slider').max=String(Math.max(100,Math.min(1000,Math.ceil(altitude/100)*100)));
  $('altitude-range-max').textContent=$('altitude-slider').max+' m · 숫자로 범위 확장';
  for(const [input,slider] of livePairs)$(slider).value=$(input).value;
  const c=readLiveTarget(),a=c.wind_angle*Math.PI/180;
  $('wind-state').textContent=`흐르는 방향 (지면 기준) · X ${number(c.wind_speed*Math.cos(a))} / Y ${number(c.wind_speed*Math.sin(a))} m/s · 0° 순풍 / 90° 측풍 / 180° 역풍`;
}
for(const [input,slider] of livePairs){
  $(slider).addEventListener('input',()=>{
    $(input).value=$(slider).value;syncLiveControls();
    // Throttle instead of debounce: a long drag still sends intermediate targets.
    if(liveTargetTimer===null)liveTargetTimer=setTimeout(()=>{liveTargetTimer=null;applyLiveTarget();},80);
  });
  $(slider).addEventListener('change',()=>{clearTimeout(liveTargetTimer);liveTargetTimer=null;applyLiveTarget();});
  $(input).addEventListener('change',()=>applyLiveTarget());
  $(input).addEventListener('keydown',event=>{if(event.key==='Enter'){event.preventDefault();applyLiveTarget();}});
}
for(const [id,angle] of [['wind-tail',0],['wind-cross',90],['wind-head',180]])$(id).addEventListener('click',()=>{
  $('live-wind-angle').value=String(angle);applyLiveTarget();
});
$('wind-calm').addEventListener('click',()=>{$('live-wind-speed').value='0';applyLiveTarget();});
syncLiveControls();
$('apply-target').addEventListener('click',()=>applyLiveTarget());$('hover-target').addEventListener('click',()=>applyLiveTarget(true));
function updateTelemetry(frame){
  if(Number.isFinite(frame.t))$('timeline-time').textContent=frame.t.toFixed(2)+' s';
  if(frame.p)$('flight-position').textContent=`시뮬레이션 ${number(frame.t)} s · X ${number(frame.p[0])} / Y ${number(frame.p[1])} / Z ${number(frame.p[2])} m`;
  $('live-speed').textContent=number(frame.v[0])+' m/s';$('live-altitude').textContent=number(frame.z)+' m';
  $('live-soc').textContent=number((frame.soc??1)*100)+' %';
  $('live-rate').textContent=frame.wall_seconds>0?number(frame.t/frame.wall_seconds)+'×':'—';
  if(frame.outside_map_fraction!==undefined){
    $('live-limits').textContent=`추진 맵 밖 ${number(frame.outside_map_fraction*100)} % · 전류 제한 ${number(frame.current_limited_fraction*100)} % · 배터리 ${number(frame.voltage)} V · 소비 ${number(frame.energy_J)} J`+
      (frame.outside_map_fraction>0?' · 미검증 추진 영역이 포함되어 있습니다.':'');
    $('live-limits').classList.toggle('bad',frame.outside_map_fraction>0);
  }
}
$('download').addEventListener('click',()=>{
  // Compact JSON keeps the maximum 20 s / 1 kHz paired log practical to reopen.
  const blob=new Blob([JSON.stringify({exported_at:new Date().toISOString(),results})],{type:'application/json'});
  const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='drone-research-results.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
});
function replayResult(){return results[$('replay-controller').value]||null;}
function setupReplay(atEnd=false){
  const kinds=Object.keys(results),current=$('replay-controller').value;
  if(!kinds.includes(current)&&kinds.length)$('replay-controller').value=kinds[0];
  for(const option of $('replay-controller').options)option.disabled=!results[option.value];
  const r=replayResult();
  for(const id of ['replay-controller','timeline','play','reuse'])$(id).disabled=running||!r;
  $('compare-record').disabled=running||!r||r.simulated_seconds<=0;
  if(!r)return;
  if(!running)replayActive=true;
  replayTime=atEnd?r.simulated_seconds:0;$('timeline').max=String(r.simulated_seconds);
  updatePath(r);if(!running)renderReplay();
}
function renderReplay(){
  const r=replayResult();if(!r)return;
  if(!replayActive){replayActive=true;updatePath(r);drawCharts();}updateFlightActions();
  const frame=ResearchResults.sample(r.trace,replayTime);if(!frame)return;
  replayTime=frame.t;const x=frame.state;$('timeline').value=String(replayTime);rotorRates=x.slice(13,17);
  updateTelemetry({t:frame.t,p:x.slice(0,3),v:x.slice(3,6),z:x[2],soc:x[17]});
  $('status').textContent=`기록 보기 · ${controllerNames[r.configuration.controller]} · ${number(frame.t)} / ${number(r.simulated_seconds)} s (새 물리 계산 아님)`;
  $('live-rate').textContent=playing?$('replay-speed').value+'× 재생':'기록 정지';
  $('replay-state').textContent=`${number(replayTime)} / ${number(r.simulated_seconds)} s · 위치 [${x.slice(0,3).map(number).join(', ')}] m · vx ${number(x[3])} m/s`;
  if(vehicle){vehicle.visible=r.profile_id==='selected-6931';vehicle.position.set(...x.slice(0,3));vehicle.quaternion.set(...x.slice(6,10));drawModel();}
  $('model-caption').textContent=r.profile_id==='selected-6931'?`${controllerNames[r.configuration.controller]} 기록 · ${SELECTED_DISPLAY_LABEL} · 로터는 시각 감속·잔상`:'단순 검증 모델 기록 · 선정안 STL은 해당 형상이 아니므로 숨김';
}
$('play').addEventListener('click',()=>{
  if(!replayResult()||running)return;
  if(replayTime>=replayResult().simulated_seconds)replayTime=0;
  setPlaying(!playing);
});
$('timeline').addEventListener('input',()=>{setPlaying(false);replayTime=Number($('timeline').value);renderReplay();});
$('replay-controller').addEventListener('change',()=>{setPlaying(false);setupReplay();});
$('view-mode').addEventListener('change',()=>drawModel());
$('import').addEventListener('click',()=>$('import-file').click());
$('import-file').addEventListener('change',async event=>{
  const file=event.target.files[0];if(!file)return;
  try{
    if(file.size>256*1024*1024)throw new Error('로그는 256 MiB 이하만 가져올 수 있습니다. 대용량 기록은 데스크톱에서 여세요.');
    const incoming=ResearchResults.validateImport(JSON.parse(await file.text()),ResearchRuntime.validateOptions);
    if(running)throw new Error('실험 실행 중에는 로그를 교체할 수 없습니다.');
    setPlaying(false);Object.keys(results).forEach(k=>delete results[k]);Object.assign(results,incoming);imported=true;
    showResults();setupReplay();$('errors').textContent='';$('status').textContent='로그를 가져왔습니다. 재생하거나 설정을 재사용할 수 있습니다. 현재 코드와 다른 버전일 수 있으므로 구현 해시를 확인하세요.';
  }catch(error){$('errors').textContent=error.message;}
  finally{event.target.value='';}
});
function reuseSettings(nextMode,observedDuration=false){
  const r=replayResult();if(!r||running)return;const c=r.configuration;
  hybridActuatorFeedback=c.hybrid_actuator_feedback??false;
  setMode(nextMode);
  $('profile').value=r.profile_id==='selected-6931'?'selected':'simple';
  $('controller').value=Object.keys(results).length===2||c.controller==='pd'?'both':c.controller;
  for(const id of ['seconds','speed','seed','feedback','scenario','altitude','preview'])$(id).value=String(c[id]);
  $('log-hz').value=String(c.log_hz);
  $('wind-speed').value=String(c.wind_speed??0);$('wind-angle').value=String(c.wind_angle??90);
  $('observe-controller').value=c.controller;
  const first=c.commands?.[0]||c;
  for(const [id,key] of [['live-target-speed','speed'],['live-target-altitude','altitude'],['live-wind-speed','wind_speed'],['live-wind-angle','wind_angle']])$(id).value=String(first[key]??(key==='wind_angle'?90:0));
  syncLiveControls();updateModeNote();
  if(observedDuration)$('seconds').value=String(r.simulated_seconds);
  recordedCommands=c.commands?.filter(command=>command.t<=Number($('seconds').value)).map(command=>({...command}))||null;
  $('scenario').querySelector('[value="schedule"]').disabled=!recordedCommands;
  if(observedDuration)$('preview').value='false';
  let mismatch=Object.keys(mismatches).find(k=>JSON.stringify(mismatches[k])===JSON.stringify(c.scales));
  if(!mismatch){
    customScales=c.scales.slice();mismatch='custom';let option=$('mismatch').querySelector('[value="custom"]');
    if(!option){option=document.createElement('option');option.value='custom';$('mismatch').append(option);}
    option.textContent='가져온 오차 배율: '+customScales.join(', ');
  }
  $('mismatch').value=mismatch;$('settings').open=true;scenarioNote(false);
  updateAircraftLabel();
  $('status').textContent=observedDuration?'기록된 목표·관측 기간·센서·기체 조건을 정밀 비교에 적용했습니다. 비예고 모드이며 실행 버튼을 눌러 계산합니다.':
    '시험 설정을 복원했습니다. 실행 시 현재 구현을 사용하므로 과거 결과와 코드 버전이 다를 수 있습니다.';
}
$('reuse').addEventListener('click',()=>reuseSettings(replayResult()?.execution?.mode==='observe'||replayResult()?.configuration.controller==='pd'?'observe':'compare'));
$('compare-record').addEventListener('click',()=>reuseSettings('compare',true));
function scenarioNote(adjust=true){
  if(mode==='observe'){
    $('seconds').min='.1';$('scenario-note').textContent=$('scenario').value==='schedule'?
      '기록한 목표·바람 일정을 재실행합니다. 새 목표를 적용하면 그 시점 이후의 일정을 대체합니다.':'초기 속도 0에서 시작하여 조작부의 목표를 추종합니다. 비행 중 목표·바람을 바꿀 수 있습니다.';return;
  }
  const scenario=$('scenario').value,min={hover:.1,step:1.1,gust:3.1,schedule:.02}[scenario];
  $('seconds').min=String(min);
  if(adjust&&Number($('seconds').value)<min)$('seconds').value=String({hover:.2,step:2,gust:4}[scenario]);
  $('scenario-note').textContent={hover:'호버: 짧은 실행 경로 확인용. 긴 과도응답 검증과는 다릅니다.',
    step:'1초에 목표 속도 변경 · 최소 1.1초. 정착 성능 평가는 더 긴 기록이 필요합니다.',
    gust:'2–3초 측풍·모멘트 외란 · 최소 3.1초. 외란 이후 복귀까지 보려면 기간을 늘리세요.',
    schedule:`선택한 앞 ${number(Number($('seconds').value))}초 구간의 목표 ${recordedCommands?.filter(c=>c.t<=Number($('seconds').value)).length||0}개를 원래 시각에 적용합니다. 원본 기록은 보존됩니다.`}[scenario];
  const solves=Math.ceil(Number($('seconds').value)/.02);
  $('scenario-note').textContent+=` 제어기마다 IPOPT 약 ${number(solves)}회. 긴 관찰 기록은 먼저 0.2–2초 구간으로 계산하세요(속도 계단 시험은 1.1초 이상).`;
}
$('scenario').addEventListener('change',()=>scenarioNote());
$('seconds').addEventListener('input',()=>scenarioNote(false));
$('profile').addEventListener('change',()=>{
  updateAircraftLabel();
  if(replayResult())return; // Editing future settings must not relabel a past log.
  if(vehicle) vehicle.visible=$('profile').value==='selected';
  drawModel();
  $('model-caption').textContent=$('profile').value==='selected'?SELECTED_DISPLAY_LABEL+' · 로터 시각 감속·잔상 · 물리 RPM 유지':'단순 시험 모델 선택됨. 선정안 STL은 이 모델의 형상이 아니므로 숨겼습니다.';
});
setMode(new URL(location.href).searchParams.get('mode')==='compare'?'compare':'observe');
async function previewSTL() {
  if(typeof THREE==='undefined') throw new Error('Three.js를 불러오지 못했습니다. 계산 기능은 계속 사용할 수 있습니다.');
  const host=$('model'),scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(42,1,.005,10000);
  scene.background=new THREE.Color(0x0a0f15);
  const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));host.append(renderer.domElement);
  scene.add(new THREE.HemisphereLight(0xe9f2ff,0x202a36,1.8));
  const lamp=new THREE.DirectionalLight(0xffffff,1.4);lamp.position.set(1,1,2);scene.add(lamp);
  const [response,geometryResponse]=await Promise.all([fetch('./assets/drone_v2.stl'),fetch('./assets/selected_geometry.json')]);
  if(!response.ok)throw new Error('STL HTTP '+response.status);
  if(!geometryResponse.ok)throw new Error('선정안 표시 형상 HTTP '+geometryResponse.status);
  const geometry=await geometryResponse.json();
  const mesh=ResearchSTL.alignSelectedGeometry(ResearchSTL.parse(await response.arrayBuffer(),geometry.cg_from_nose_m),geometry);
  vehicle=ResearchSTL.createVehicle(mesh,THREE);scene.add(vehicle);
  vehicle.position.set(0,0,Number($('altitude').value));vehicle.quaternion.set(0,-Math.SQRT1_2,0,Math.SQRT1_2);
  const axes=new THREE.AxesHelper(.3);scene.add(axes);
  let pathLine=null,pathCenter=new THREE.Vector3(),pathRadius=1.3,livePositions=null,liveCount=0,liveBounds=new THREE.Box3();
  const ground=new THREE.GridHelper(200,40,0x30404e,0x1d2a36);ground.rotation.x=Math.PI/2;scene.add(ground);
  // A labelled visual reference plane makes translation visible in chase view.
  // It is not terrain, and neither this grid nor the camera enters the plant.
  const flightGrid=new THREE.GridHelper(200,200,0x375366,0x20323f);flightGrid.rotation.x=Math.PI/2;
  flightGrid.position.z=Number($('altitude').value)-1;scene.add(flightGrid);
  const altitudeLine=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3()]),
    new THREE.LineBasicMaterial({color:0x6d8796,transparent:true,opacity:.65}));scene.add(altitudeLine);
  updatePath=function(result){
    if(pathLine){scene.remove(pathLine);pathLine.geometry.dispose();pathLine.material.dispose();pathLine=null;}
    livePositions=null;liveCount=0;
    if(!result){
      axes.position.set(0,0,Number($('altitude').value));liveBounds.makeEmpty();pathCenter.copy(axes.position);pathRadius=1.3;
      flightGrid.position.z=Number($('altitude').value)-1;
      livePositions=new Float32Array(6002*3);
      const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(livePositions,3));geometry.setDrawRange(0,0);
      pathLine=new THREE.Line(geometry,new THREE.LineBasicMaterial({color:mode==='observe'?colors[$('observe-controller').value]:colors.hybrid}));
      pathLine.frustumCulled=false;scene.add(pathLine);return;
    }
    const positions=result.trace.filter((_,i)=>i%Math.max(1,Math.ceil(result.trace.length/5000))===0||i===result.trace.length-1)
      .map(row=>new THREE.Vector3(...row.state.slice(0,3)));
    flightGrid.position.z=result.trace[0].state[2]-1;
    const geometry=new THREE.BufferGeometry().setFromPoints(positions);
    pathLine=new THREE.Line(geometry,new THREE.LineBasicMaterial({color:colors[result.configuration.controller]}));scene.add(pathLine);
    const box=new THREE.Box3().setFromPoints(positions);box.getCenter(pathCenter);pathRadius=Math.max(1.3,box.getSize(new THREE.Vector3()).length()*1.6);
    axes.position.copy(positions[0]);drawModel();
  };
  appendLivePath=function(position){
    if(!livePositions||!pathLine)return;
    const index=Math.min(liveCount,6001);livePositions.set(position,index*3);liveCount=Math.min(liveCount+1,6002);
    pathLine.geometry.setDrawRange(0,liveCount);pathLine.geometry.attributes.position.needsUpdate=true;
    liveBounds.expandByPoint(new THREE.Vector3(...position));liveBounds.getCenter(pathCenter);
    pathRadius=Math.max(1.3,liveBounds.getSize(new THREE.Vector3()).length()*1.6);
  };
  let azimuth=-.9,elevation=.35,drag=null,zoom=1,following=true,noseView=false;
  const pan=new THREE.Vector3(),fixedTarget=vehicle.position.clone();
  function setFollowing(value){
    following=value;fixedTarget.copy(vehicle.position);
    $('follow').setAttribute('aria-pressed',String(value));
    $('follow').textContent=value?'따라가기 ON (F)':'따라가기 OFF (F)';drawModel();
  }
  $('follow').addEventListener('click',()=>setFollowing(!following));
  for(const [id,az,el] of [['camera-back',Math.PI,.15],['camera-side',-Math.PI/2,.15],['camera-top',0,Math.PI/2-.01]]){
    $(id).addEventListener('click',()=>{azimuth=az;elevation=el;noseView=false;pan.set(0,0,0);drawModel();});
  }
  $('camera-nose').addEventListener('click',()=>{noseView=true;pan.set(0,0,0);setFollowing(true);});
  $('camera-reset').addEventListener('click',()=>{azimuth=-.9;elevation=.35;zoom=1;noseView=false;pan.set(0,0,0);setFollowing(true);});
  host.addEventListener('wheel',event=>{event.preventDefault();zoom=Math.max(.35,Math.min(6,zoom*Math.exp(event.deltaY*.001)));drawModel();},{passive:false});
  host.addEventListener('pointerdown',event=>{
    drag=[event.clientX,event.clientY];host.setPointerCapture(event.pointerId);$('flight-view').focus({preventScroll:true});event.preventDefault();
  });
  host.addEventListener('pointermove',event=>{
    if(!drag)return;
    const dx=event.clientX-drag[0],dy=event.clientY-drag[1];
    if(event.shiftKey){
      // Translate the camera target in screen axes; never translate the aircraft.
      const scale=2*camera.position.distanceTo(vehicle.position.clone().add(pan))*Math.tan(camera.fov*Math.PI/360)/Math.max(1,host.clientHeight);
      pan.addScaledVector(new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld,0),-dx*scale);
      pan.addScaledVector(new THREE.Vector3().setFromMatrixColumn(camera.matrixWorld,1),dy*scale);
    }else{noseView=false;azimuth-=dx*.01;elevation=Math.max(-1.55,Math.min(1.55,elevation+dy*.01));}
    drag=[event.clientX,event.clientY];drawModel();
  });
  host.addEventListener('pointerup',()=>drag=null);host.addEventListener('pointercancel',()=>drag=null);
  drawModel=function() {
    const w=host.clientWidth,h=host.clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();
    const entire=$('view-mode').value==='path'&&pathLine,world=$('view-mode').value==='world';
    const tracked=following?vehicle.position:fixedTarget;
    const target=(entire?pathCenter:world?new THREE.Vector3(tracked.x,tracked.y,tracked.z*.5):tracked).clone().add(pan);
    const radius=(entire?pathRadius:world?Math.max(4,vehicle.position.z*1.8):3)*zoom;
    flightGrid.visible=!world;
    const line=altitudeLine.geometry.attributes.position;
    line.setXYZ(0,vehicle.position.x,vehicle.position.y,0);line.setXYZ(1,...vehicle.position.toArray());line.needsUpdate=true;
    altitudeLine.frustumCulled=false;
    camera.up.set(0,0,1);
    if(noseView){
      camera.position.copy(target).addScaledVector(new THREE.Vector3(1,0,0).applyQuaternion(vehicle.quaternion),radius);
      camera.up.copy(new THREE.Vector3(0,1,0).applyQuaternion(vehicle.quaternion));
    }else camera.position.set(target.x+radius*Math.cos(elevation)*Math.cos(azimuth),target.y+radius*Math.cos(elevation)*Math.sin(azimuth),target.z+radius*Math.sin(elevation));
    camera.lookAt(target);
    renderer.render(scene,camera);
  };
  new ResizeObserver(drawModel).observe(host);
  drawModel();$('model-caption').textContent=SELECTED_DISPLAY_LABEL+' · 4개 로터 · 시각 감속·잔상 (물리 RPM 유지)';
  if(replayResult())setupReplay();
}
previewSTL().catch(error=>$('model-caption').textContent=error.message);
