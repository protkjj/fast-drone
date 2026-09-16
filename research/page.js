'use strict';
const $=id=>document.getElementById(id);
const results={};
let worker=null, vehicle=null, running=false, drawModel=()=>{},updatePath=()=>{};
let playing=false,replayTime=0,imported=false,customScales=null;
const mismatches={nominal:[1,1,1,1,1],mass:[1.1,1,1,1,1],inertia:[1,1.15,1.15,1.15,1],
  prop:[1,1,1,1,.9],combined:[1.1,1.15,1.15,1.15,.9]};
if(matchMedia('(max-width:850px)').matches)$('settings').open=false;
let rotorRates=[0,0,0,0],rotorFrame=null,lastRotorFrame=0;
function animateModel(now){
  rotorFrame=null;
  if(!running&&!playing)return;
  if(now-lastRotorFrame>=1000/30){
    const dt=Math.min(.1,(now-lastRotorFrame)/1000);lastRotorFrame=now;
    if(playing){
      replayTime=Math.min(replayResult().simulated_seconds,replayTime+dt*Number($('replay-speed').value));
      renderReplay();if(replayTime>=replayResult().simulated_seconds)setPlaying(false);
    }
    if(vehicle&&vehicle.visible){ResearchSTL.animateRotors(vehicle,rotorRates,dt,running||playing);drawModel();}
  }
  if(running||playing)rotorFrame=requestAnimationFrame(animateModel);
}
function startFrames(){if(rotorFrame===null){lastRotorFrame=performance.now();rotorFrame=requestAnimationFrame(animateModel);}}
function setPlaying(value){
  playing=value;$('play').textContent=value?'일시정지':'재생';
  if(value)startFrames();else if(vehicle){ResearchSTL.animateRotors(vehicle,rotorRates,0,false);drawModel();}
}
const colors={hybrid:'#59c8f5',nmpc:'#db9bff'};
function setBusy(value) {
  running=value; $('run').disabled=value; $('stop').disabled=!value;
  document.querySelectorAll('.controls input,.controls select').forEach(el=>el.disabled=value);
  $('import').disabled=value;$('reuse').disabled=value||!replayResult();
  $('play').disabled=value||!replayResult();$('timeline').disabled=value||!replayResult();
  $('replay-controller').disabled=value||!replayResult();
  if(value){setPlaying(false);if(vehicle)vehicle.visible=$('profile').value==='selected';startFrames();}
  if(!value&&vehicle){ResearchSTL.animateRotors(vehicle,rotorRates,0,false);drawModel();}
}
const number=value=>!Number.isFinite(value)?'—':value!==0&&Math.abs(value)<.001?value.toExponential(2):value.toLocaleString('ko-KR',{maximumFractionDigits:3});
function showResults() {
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
  addRow('실행 결과',['hybrid','nmpc'].map(k=>results[k]?(results[k].failure||(results[k].status==='stopped'?'사용자 중지 · 부분 결과':null)||
    (results[k].metrics.solver_failures?'풀이 실패 포함 · 비교 해석 주의':'계산 완료')):'—'));
  addRow('시뮬레이션 시간 · s',['hybrid','nmpc'].map(k=>number(results[k]?.simulated_seconds)));
  addRow('실제 총 계산 시간 · s',['hybrid','nmpc'].map(k=>number(results[k]?.wall_seconds)));
  for(const [label,key,scale=1] of rows) addRow(label,['hybrid','nmpc'].map(k=>{
    const value=results[k]?.metrics[key];return number(value==null?null:value*scale);
  }));
  addRow('속도 입력 / 외란 종료 포함',['hybrid','nmpc'].map(k=>{
    const r=results[k];if(!r)return '—';const e=r.event_coverage;
    return (r.configuration.scenario==='hover'?'해당 없음':e?.speed_step?'입력 포함':'입력 미도달')+
      (r.configuration.scenario==='gust'?(e?.gust_completed?' / 외란 종료 포함':' / 외란 미완료'):'');
  }));
  addRow('NMPC / INDI / IMU 횟수',['hybrid','nmpc'].map(k=>{const c=results[k]?.counts;return c?`${c.nmpc} / ${c.indi} / ${c.imu}`:'—';}));
  addRow('GPS 융합 / 재적분 IMU 횟수',['hybrid','nmpc'].map(k=>{const c=results[k]?.counts;return c?`${c.gps_fused} / ${c.gps_replayed_imu}`:'—';}));
  drawChart('velocity-chart',point=>point.v[0],point=>point.reference[0]);
  drawChart('altitude-chart',point=>point.z,point=>point.reference[3]);
  $('download').disabled=Object.keys(results).length===0;
  const result=Object.values(results)[0];
  $('result-context').textContent=result?`${imported?'가져온 기록 · ':'현재 실행 · '}${result.profile_id} · ${result.configuration.scenario} · seed ${result.configuration.seed} · ${result.configuration.preview?'미래 목표 예고':'비예고'} · 구현 ${result.implementation?.input_sha256?.slice(0,12)||'알 수 없음'}\n제한 비율은 각 tick에 4개 모터 중 하나라도 해당 제한이 작동한 비율입니다. 모터 추종 제한은 전류·전압·회생 제동 불가를 포함하며 속도 오차 자체와는 다릅니다.`:'';
}
function drawChart(id,value,reference) {
  const svg=$(id), ns='http://www.w3.org/2000/svg';svg.replaceChildren();
  const all=Object.values(results).flatMap(r=>r.trace);if(!all.length)return;
  let lo=Infinity,hi=-Infinity,time=.001;
  for(const point of all){lo=Math.min(lo,value(point),reference(point));hi=Math.max(hi,value(point),reference(point));time=Math.max(time,point.t);}
  const margin=Math.max(.05,(hi-lo)*.12);lo-=margin;hi+=margin;
  const sx=t=>55+t/time*545,sy=v=>185-(v-lo)/(hi-lo)*165;
  const element=(name,attributes,text)=>{const node=document.createElementNS(ns,name);Object.entries(attributes).forEach(([k,v])=>node.setAttribute(k,v));if(text!==undefined)node.textContent=text;svg.append(node);return node;};
  for(let i=0;i<5;i++) {
    const v=lo+(hi-lo)*i/4,y=sy(v);
    element('line',{x1:55,x2:600,y1:y,y2:y,stroke:'#30404e'});
    element('text',{x:48,y:y+4,fill:'#aebac6','font-size':11,'text-anchor':'end'},number(v));
    element('text',{x:sx(time*i/4),y:210,fill:'#aebac6','font-size':11,'text-anchor':'middle'},number(time*i/4));
  }
  // Keep long 1 kHz logs responsive. The exported log retains every sample.
  const path=(trace,fn)=>trace.filter((_,i)=>i%Math.max(1,Math.ceil(trace.length/2000))===0||i===trace.length-1)
    .map((point,i)=>`${i?'L':'M'}${sx(point.t).toFixed(2)},${sy(fn(point)).toFixed(2)}`).join(' ');
  element('path',{d:path(Object.values(results)[0].trace,reference),fill:'none',stroke:'#aebac6','stroke-dasharray':'5 4','stroke-width':1.5});
  for(const [kind,result] of Object.entries(results)) element('path',{d:path(result.trace,value),fill:'none',stroke:colors[kind],'stroke-width':2});
}
$('run').addEventListener('click',()=>{
  const seconds=Number($('seconds').value), speed=Number($('speed').value),seed=Number($('seed').value);
  const mismatch=mismatches[$('mismatch').value]||customScales;
  let options;
  try{options=ResearchRuntime.validateOptions({seconds,speed,seed,feedback:$('feedback').value,scenario:$('scenario').value,
    scales:mismatch,altitude:Number($('altitude').value),preview:$('preview').value==='true',log_hz:Number($('log-hz').value)});}
  catch(error){$('errors').textContent=error.message;$('settings').open=true;return;}
  setPlaying(false);imported=false;
  Object.keys(results).forEach(k=>delete results[k]);showResults();$('errors').textContent='';
  updatePath(null);
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
        $('status').textContent=`${m.controller==='hybrid'?'Hybrid':'NMPC 단독'} · ${number(m.t)} / ${number(m.total)} s\n실제 vx ${number(m.v[0])} m/s · 고도 ${number(m.z)} m · NMPC ${m.solves}회`;
        $('progress').value=m.t/m.total;
        if(vehicle&&m.q) {vehicle.quaternion.set(...m.q);if(m.p)vehicle.position.set(...m.p);drawModel();}
      }
      if(m.type==='result') {
        results[m.result.configuration.controller]=m.result;showResults();
        $('replay-controller').value=m.result.configuration.controller;setupReplay();
        rotorRates=m.result.final.slice(13,17);
        if(vehicle){vehicle.position.set(...m.result.final.slice(0,3));vehicle.quaternion.set(...m.result.final.slice(6,10));drawModel();}
      }
      if(m.type==='done') {
        setBusy(false);setupReplay();$('status').textContent=m.stopped?(m.count?'중지됨. 현재까지 계산된 부분 결과도 보존했습니다.':'모델 준비 중에 중지했습니다. 아직 계산된 결과는 없습니다.'):'계산 완료. 아래 시간축으로 재생하고 실패·제한·맵 적용 범위를 함께 확인하세요.';
      }
      if(m.type==='error') {$('errors').textContent=m.text;setBusy(false);}
    };
  }
  worker.postMessage({type:'run',profile:$('profile').value,
    controllers:$('controller').value==='both'?['hybrid','nmpc']:[$('controller').value],
    options});
});
$('stop').addEventListener('click',()=>{worker?.postMessage({type:'stop'});$('status').textContent='중지 요청됨. 현재 IPOPT 풀이가 끝나는 시점에 안전하게 중지합니다.';$('stop').disabled=true;});
$('download').addEventListener('click',()=>{
  // Compact JSON keeps the maximum 20 s / 1 kHz paired log practical to reopen.
  const blob=new Blob([JSON.stringify({exported_at:new Date().toISOString(),results})],{type:'application/json'});
  const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='drone-research-results.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
});
function replayResult(){return results[$('replay-controller').value]||null;}
function setupReplay(){
  const kinds=Object.keys(results),current=$('replay-controller').value;
  if(!kinds.includes(current)&&kinds.length)$('replay-controller').value=kinds[0];
  for(const option of $('replay-controller').options)option.disabled=!results[option.value];
  const r=replayResult();
  for(const id of ['replay-controller','timeline','play','reuse'])$(id).disabled=running||!r;
  if(!r)return;
  replayTime=0;$('timeline').max=String(r.simulated_seconds);
  updatePath(r);if(!running)renderReplay();
}
function renderReplay(){
  const r=replayResult();if(!r)return;
  const frame=ResearchResults.sample(r.trace,replayTime);if(!frame)return;
  replayTime=frame.t;const x=frame.state;$('timeline').value=String(replayTime);rotorRates=x.slice(13,17);
  $('replay-state').textContent=`${number(replayTime)} / ${number(r.simulated_seconds)} s · 위치 [${x.slice(0,3).map(number).join(', ')}] m · vx ${number(x[3])} m/s`;
  if(vehicle){vehicle.visible=r.profile_id==='selected-6931';vehicle.position.set(...x.slice(0,3));vehicle.quaternion.set(...x.slice(6,10));drawModel();}
  $('model-caption').textContent=r.profile_id==='selected-6931'?'drone_v2.stl · 기록된 위치·자세 · 로터는 시각 감속·잔상':'단순 검증 모델 기록 · 선정안 STL은 해당 형상이 아니므로 숨김';
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
$('reuse').addEventListener('click',()=>{
  const r=replayResult();if(!r||running)return;const c=r.configuration;
  $('profile').value=r.profile_id==='selected-6931'?'selected':'simple';
  $('controller').value=Object.keys(results).length===2?'both':c.controller;
  for(const id of ['seconds','speed','seed','feedback','scenario','altitude','preview'])$(id).value=String(c[id]);
  $('log-hz').value=String(c.log_hz);
  let mismatch=Object.keys(mismatches).find(k=>JSON.stringify(mismatches[k])===JSON.stringify(c.scales));
  if(!mismatch){
    customScales=c.scales.slice();mismatch='custom';let option=$('mismatch').querySelector('[value="custom"]');
    if(!option){option=document.createElement('option');option.value='custom';$('mismatch').append(option);}
    option.textContent='가져온 오차 배율: '+customScales.join(', ');
  }
  $('mismatch').value=mismatch;$('settings').open=true;scenarioNote(false);
  $('status').textContent='시험 설정을 복원했습니다. 실행 시 현재 구현을 사용하므로 과거 결과와 코드 버전이 다를 수 있습니다.';
});
function scenarioNote(adjust=true){
  const scenario=$('scenario').value,min={hover:.1,step:1.1,gust:3.1}[scenario];
  $('seconds').min=String(min);
  if(adjust&&Number($('seconds').value)<min)$('seconds').value=String({hover:.2,step:2,gust:4}[scenario]);
  $('scenario-note').textContent={hover:'호버: 짧은 실행 경로 확인용. 긴 과도응답 검증과는 다릅니다.',
    step:'1초에 목표 속도 변경 · 최소 1.1초. 정착 성능 평가는 더 긴 기록이 필요합니다.',
    gust:'2–3초 측풍·모멘트 외란 · 최소 3.1초. 외란 이후 복귀까지 보려면 기간을 늘리세요.'}[scenario];
}
$('scenario').addEventListener('change',()=>scenarioNote());
$('profile').addEventListener('change',()=>{
  if(replayResult())return; // Editing future settings must not relabel a past log.
  if(vehicle) vehicle.visible=$('profile').value==='selected';
  drawModel();
  $('model-caption').textContent=$('profile').value==='selected'?'drone_v2.stl · 로터 시각 감속·잔상 · 물리 RPM 유지':'단순 시험 모델 선택됨. 선정안 STL은 이 모델의 형상이 아니므로 숨겼습니다.';
});
async function previewSTL() {
  if(typeof THREE==='undefined') throw new Error('Three.js를 불러오지 못했습니다. 계산 기능은 계속 사용할 수 있습니다.');
  const host=$('model'),scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(42,1,.005,10000);
  scene.background=new THREE.Color(0x0a0f15);
  const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));host.append(renderer.domElement);
  scene.add(new THREE.HemisphereLight(0xe9f2ff,0x202a36,1.8));
  const lamp=new THREE.DirectionalLight(0xffffff,1.4);lamp.position.set(1,1,2);scene.add(lamp);
  const response=await fetch('./assets/drone_v2.stl');if(!response.ok)throw new Error('STL HTTP '+response.status);
  const mesh=ResearchSTL.parse(await response.arrayBuffer());
  vehicle=ResearchSTL.createVehicle(mesh,THREE);scene.add(vehicle);
  const axes=new THREE.AxesHelper(.3);scene.add(axes);
  let pathLine=null,pathCenter=new THREE.Vector3(),pathRadius=1.3;
  const ground=new THREE.GridHelper(200,40,0x30404e,0x1d2a36);ground.rotation.x=Math.PI/2;scene.add(ground);
  updatePath=function(result){
    if(pathLine){scene.remove(pathLine);pathLine.geometry.dispose();pathLine.material.dispose();pathLine=null;}
    if(!result){axes.position.set(0,0,Number($('altitude').value));return;}
    const positions=result.trace.filter((_,i)=>i%Math.max(1,Math.ceil(result.trace.length/5000))===0||i===result.trace.length-1)
      .map(row=>new THREE.Vector3(...row.state.slice(0,3)));
    const geometry=new THREE.BufferGeometry().setFromPoints(positions);
    pathLine=new THREE.Line(geometry,new THREE.LineBasicMaterial({color:colors[result.configuration.controller]}));scene.add(pathLine);
    const box=new THREE.Box3().setFromPoints(positions);box.getCenter(pathCenter);pathRadius=Math.max(1.3,box.getSize(new THREE.Vector3()).length()*1.6);
    axes.position.copy(positions[0]);drawModel();
  };
  let azimuth=-.9,elevation=.35,drag=null;
  host.addEventListener('pointerdown',event=>{drag=[event.clientX,event.clientY];host.setPointerCapture(event.pointerId);});
  host.addEventListener('pointermove',event=>{if(!drag)return;azimuth-=(event.clientX-drag[0])*.01;elevation=Math.max(-1.3,Math.min(1.3,elevation+(event.clientY-drag[1])*.01));drag=[event.clientX,event.clientY];drawModel();});
  host.addEventListener('pointerup',()=>drag=null);host.addEventListener('pointercancel',()=>drag=null);
  drawModel=function() {
    const w=host.clientWidth,h=host.clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();
    const entire=$('view-mode').value==='path'&&pathLine,target=entire?pathCenter:vehicle.position,radius=entire?pathRadius:1.3;
    camera.up.set(0,0,1);camera.position.set(target.x+radius*Math.cos(elevation)*Math.cos(azimuth),target.y+radius*Math.cos(elevation)*Math.sin(azimuth),target.z+radius*Math.sin(elevation));camera.lookAt(target);
    renderer.render(scene,camera);
  };
  new ResizeObserver(drawModel).observe(host);
  drawModel();$('model-caption').textContent='drone_v2.stl · 4개 로터 · 시각 감속·잔상 (물리 RPM 유지)';
  if(replayResult())setupReplay();
}
previewSTL().catch(error=>$('model-caption').textContent=error.message);
