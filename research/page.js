'use strict';
const $=id=>document.getElementById(id);
const results={};
let worker=null, vehicle=null, running=false, drawModel=()=>{};
const colors={hybrid:'#59c8f5',nmpc:'#db9bff'};
function setBusy(value) {
  running=value; $('run').disabled=value; $('stop').disabled=!value;
  document.querySelectorAll('.controls input,.controls select').forEach(el=>el.disabled=value);
}
const number=value=>!Number.isFinite(value)?'—':value!==0&&Math.abs(value)<.001?value.toExponential(2):value.toLocaleString('ko-KR',{maximumFractionDigits:3});
function showResults() {
  const rows=[['속도 RMSE · m/s','velocity_rmse_mps'],['고도 RMSE · m','altitude_rmse_m'],
    ['추정 위치 RMSE · m','position_estimation_rmse_m'],['소비 전기에너지 · J','electrical_energy_J'],
    ['명령 포화 비율 · %','saturation_fraction',100],['추진 맵 밖 비율 · %','outside_prop_map_fraction',100],
    ['풀이 실패 횟수','solver_failures'],['풀이 p50 · ms','solve_ms_p50'],['풀이 p95 · ms','solve_ms_p95'],
    ['풀이 최대 · ms (첫 풀이 포함)','solve_ms_max'],['20 ms 초과 횟수','deadline_misses_20ms'],
    ['모터 에너지 잔차 최대 · W','motor_energy_residual_W'],['버스 전압 잔차 최대 · V','bus_residual_V']];
  const body=$('metrics'); body.replaceChildren();
  function addRow(label,values) {
    const tr=document.createElement('tr'), title=document.createElement('td'); title.textContent=label;tr.append(title);
    values.forEach(text=>{const td=document.createElement('td');td.textContent=text;tr.append(td);});body.append(tr);
  }
  addRow('실행 결과',['hybrid','nmpc'].map(k=>results[k]?(results[k].failure||
    (results[k].metrics.solver_failures?'풀이 실패 포함 · 비교 해석 주의':'계산 완료')):'—'));
  addRow('시뮬레이션 시간 · s',['hybrid','nmpc'].map(k=>number(results[k]?.simulated_seconds)));
  addRow('실제 총 계산 시간 · s',['hybrid','nmpc'].map(k=>number(results[k]?.wall_seconds)));
  for(const [label,key,scale=1] of rows) addRow(label,['hybrid','nmpc'].map(k=>number(results[k]?.metrics[key]*scale)));
  addRow('NMPC / INDI / IMU 횟수',['hybrid','nmpc'].map(k=>{const c=results[k]?.counts;return c?`${c.nmpc} / ${c.indi} / ${c.imu}`:'—';}));
  addRow('GPS 융합 / 재적분 IMU 횟수',['hybrid','nmpc'].map(k=>{const c=results[k]?.counts;return c?`${c.gps_fused} / ${c.gps_replayed_imu}`:'—';}));
  drawChart('velocity-chart',point=>point.v[0],point=>point.reference[0]);
  drawChart('altitude-chart',point=>point.z,point=>point.reference[3]);
  $('download').disabled=Object.keys(results).length===0;
}
function drawChart(id,value,reference) {
  const svg=$(id), ns='http://www.w3.org/2000/svg';svg.replaceChildren();
  const all=Object.values(results).flatMap(r=>r.trace);if(!all.length)return;
  const values=all.flatMap(point=>[value(point),reference(point)]);
  let lo=Math.min(...values),hi=Math.max(...values);const margin=Math.max(.05,(hi-lo)*.12);lo-=margin;hi+=margin;
  const time=Math.max(...all.map(p=>p.t)), sx=t=>55+t/time*545, sy=v=>185-(v-lo)/(hi-lo)*165;
  const element=(name,attributes,text)=>{const node=document.createElementNS(ns,name);Object.entries(attributes).forEach(([k,v])=>node.setAttribute(k,v));if(text!==undefined)node.textContent=text;svg.append(node);return node;};
  for(let i=0;i<5;i++) {
    const v=lo+(hi-lo)*i/4,y=sy(v);
    element('line',{x1:55,x2:600,y1:y,y2:y,stroke:'#30404e'});
    element('text',{x:48,y:y+4,fill:'#aebac6','font-size':11,'text-anchor':'end'},number(v));
    element('text',{x:sx(time*i/4),y:210,fill:'#aebac6','font-size':11,'text-anchor':'middle'},number(time*i/4));
  }
  const path=(trace,fn)=>trace.map((point,i)=>`${i?'L':'M'}${sx(point.t).toFixed(2)},${sy(fn(point)).toFixed(2)}`).join(' ');
  element('path',{d:path(Object.values(results)[0].trace,reference),fill:'none',stroke:'#aebac6','stroke-dasharray':'5 4','stroke-width':1.5});
  for(const [kind,result] of Object.entries(results)) element('path',{d:path(result.trace,value),fill:'none',stroke:colors[kind],'stroke-width':2});
}
$('run').addEventListener('click',()=>{
  const seconds=Number($('seconds').value), speed=Number($('speed').value),seed=Number($('seed').value);
  if(!Number.isFinite(seconds)||seconds<=0||seconds>120||!Number.isFinite(speed)||speed<0||speed>100||!Number.isInteger(seed)||seed<0||seed>4294967295) {
    $('errors').textContent='기간·목표 속도·seed의 범위를 확인하세요.';return;
  }
  const mismatch={nominal:[1,1,1,1,1],mass:[1.1,1,1,1,1],inertia:[1,1.15,1.15,1.15,1],
                  prop:[1,1,1,1,.9],combined:[1.1,1.15,1.15,1.15,.9]}[$('mismatch').value];
  Object.keys(results).forEach(k=>delete results[k]);showResults();$('errors').textContent='';
  setBusy(true);$('progress').value=0;
  if(!worker) {
    worker=new Worker('./worker.js');
    worker.onerror=event=>{$('errors').textContent=event.message;setBusy(false);};
    worker.onmessage=event=>{
      const m=event.data;
      if(m.type==='status') $('status').textContent=m.text;
      if(m.type==='progress') {
        $('status').textContent=`${m.controller==='hybrid'?'Hybrid':'NMPC 단독'} · ${number(m.t)} / ${number(m.total)} s\n실제 vx ${number(m.v[0])} m/s · 고도 ${number(m.z)} m · NMPC ${m.solves}회`;
        $('progress').value=m.t/m.total;
        if(vehicle&&m.q) {vehicle.quaternion.set(...m.q);drawModel();}
      }
      if(m.type==='result') {results[m.result.configuration.controller]=m.result;showResults();}
      if(m.type==='done') {
        setBusy(false);$('status').textContent=m.stopped?'중지됨. 완료된 계산 결과를 보존했습니다.':'계산 완료. 실패·포화·맵 적용 범위·실제 계산 시간을 함께 확인하세요.';
      }
      if(m.type==='error') {$('errors').textContent=m.text;setBusy(false);}
    };
  }
  worker.postMessage({type:'run',profile:$('profile').value,
    controllers:$('controller').value==='both'?['hybrid','nmpc']:[$('controller').value],
    options:{seconds,speed,seed,feedback:$('feedback').value,scenario:$('scenario').value,scales:mismatch}});
});
$('stop').addEventListener('click',()=>{worker?.postMessage({type:'stop'});$('status').textContent='중지 요청됨. 현재 IPOPT 풀이가 끝나는 시점에 안전하게 중지합니다.';$('stop').disabled=true;});
$('download').addEventListener('click',()=>{
  const blob=new Blob([JSON.stringify({exported_at:new Date().toISOString(),results},null,2)],{type:'application/json'});
  const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='drone-research-results.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
});
$('profile').addEventListener('change',()=>{
  if(vehicle) vehicle.visible=$('profile').value==='selected';
  drawModel();
  $('model-caption').textContent=$('profile').value==='selected'?'drone_v2.stl · 선정안 외형 · 무게중심 기준 · 마우스로 드래그':'단순 시험 모델 선택됨. 선정안 STL은 이 모델의 형상이 아니므로 숨겼습니다.';
});
async function previewSTL() {
  if(typeof THREE==='undefined') throw new Error('Three.js를 불러오지 못했습니다. 계산 기능은 계속 사용할 수 있습니다.');
  const host=$('model'),scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(42,1,.005,100);
  scene.background=new THREE.Color(0x0a0f15);
  const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));host.append(renderer.domElement);
  scene.add(new THREE.HemisphereLight(0xe9f2ff,0x202a36,1.8));
  const lamp=new THREE.DirectionalLight(0xffffff,1.4);lamp.position.set(1,1,2);scene.add(lamp);
  const response=await fetch('./assets/drone_v2.stl');if(!response.ok)throw new Error('STL HTTP '+response.status);
  const mesh=ResearchSTL.parse(await response.arrayBuffer());
  const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(mesh.positions,3));geometry.computeVertexNormals();
  vehicle=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:0xb5c8d5,metalness:.22,roughness:.55,side:THREE.DoubleSide}));scene.add(vehicle);
  const axes=new THREE.AxesHelper(.16);scene.add(axes);
  let azimuth=-.9,elevation=.35,drag=null;
  host.addEventListener('pointerdown',event=>{drag=[event.clientX,event.clientY];host.setPointerCapture(event.pointerId);});
  host.addEventListener('pointermove',event=>{if(!drag)return;azimuth-=(event.clientX-drag[0])*.01;elevation=Math.max(-1.3,Math.min(1.3,elevation+(event.clientY-drag[1])*.01));drag=[event.clientX,event.clientY];drawModel();});
  host.addEventListener('pointerup',()=>drag=null);host.addEventListener('pointercancel',()=>drag=null);
  drawModel=function() {
    const w=host.clientWidth,h=host.clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();
    camera.up.set(0,0,1);camera.position.set(1.3*Math.cos(elevation)*Math.cos(azimuth),1.3*Math.cos(elevation)*Math.sin(azimuth),1.3*Math.sin(elevation));camera.lookAt(0,0,0);
    renderer.render(scene,camera);
  };
  new ResizeObserver(drawModel).observe(host);
  drawModel();$('model-caption').textContent=`drone_v2.stl · ${mesh.triangles.toLocaleString()} triangles · CG 기준 · 마우스로 드래그`;
}
previewSTL().catch(error=>$('model-caption').textContent=error.message);
