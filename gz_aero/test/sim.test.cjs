const test = require('node:test');
const assert = require('node:assert/strict');
const {simulator} = require('./sim_harness.cjs');

test('기본 목표는 약 300 km/h와 200 m이며 실제 속도는 초기화하지 않는다',()=>{
  const fs=require('node:fs'),path=require('node:path');
  const html=fs.readFileSync(path.join(__dirname,'../../results/flight_sim.html'),'utf8');
  const speed=html.match(/id="spd"[^>]*value="([^"]+)"/)[1];
  const altitude=html.match(/id="alt"[^>]*value="([^"]+)"/)[1];
  assert.equal(Number(speed),83.3);assert.ok(Math.abs(Number(speed)*3.6-300)<.2);
  assert.equal(Number(altitude),200);assert.match(html,/id="spd"[^>]*step="0\.1"/);
  const sim=simulator(),state=sim.run(`$('#spd').value=${JSON.stringify(speed)};reset();
    ({velocity:X.slice(3,6),reference:cmdSpd,target:cmdNow().target});`);
  assert.deepEqual(Array.from(state.velocity),[0,0,0]);assert.equal(state.reference,0);assert.equal(state.target,83.3);
});

test('역방향·사선 유입 공력은 병진 운동 에너지를 생성하지 않는다',()=>{
  const sim=simulator();
  const powers=sim.run(`[[ -20,0,0],[-20,5,3],[0,5,3],[20,5,3]].map(v=>{
    const x=Array(17).fill(0);x[2]=20;x[9]=1;x.splice(3,3,...v);
    const dx=xdot(x,[0,0,0,0],[0,0,0],P0);
    return P0.mass*(dx[3]*v[0]+dx[4]*v[1]+(dx[5]+P0.g)*v[2]);
  })`);
  powers.forEach(power=>assert.ok(power<=1e-9,`공력 일률 ${power} W`));
});

test('STL 외형은 상태의 위치·자세만 따르며 기존 물리 상태를 바꾸지 않는다', () => {
  const sim=simulator();
  const result=sim.run(`const before=Array.from(X), calls={};
    const display={position:{set:(...v)=>calls.p=v},quaternion:{set:(...v)=>calls.q=v}};
    syncVehiclePose(display,X);
    ({before,after:Array.from(X),position:calls.p,q:calls.q,mass:P.mass,
      parser:typeof ResearchSTL.parse,url:STL_URL});`);
  assert.deepEqual(Array.from(result.before),Array.from(result.after));
  assert.deepEqual(Array.from(result.position),Array.from(result.before).slice(0,3));
  assert.deepEqual(Array.from(result.q),Array.from(result.before).slice(6,10));
  assert.equal(result.mass,8);assert.equal(result.parser,'function');
  assert.equal(result.url,'../research/assets/drone_v2.stl');
});

test('Python 기준 궤적 및 LQR 보간과 일치한다', () => {
  const sim = simulator();
  assert.ok(sim.run('selfCheck()') < 1e-9);
  assert.ok(sim.run('lqrCheck()') < 1e-9);
  const maxTiltError=sim.run(`Math.max(...Array.from({length:171},(_,i)=>{
    const [x]=lqrPick(i/2);
    const expected=Math.acos(Math.max(-1,Math.min(1,2*(x[6]*x[8]-x[7]*x[9]))))*180/Math.PI;
    return Math.abs(trimTiltDeg(i/2)-expected);
  }))`);
  assert.ok(maxTiltError<1e-6);
});

test('30/60/144 FPS에서 시동과 비행을 포함한 같은 3초 궤적을 계산한다', () => {
  const ends = [30, 60, 144].map(fps => {
    const sim = simulator();
    return sim.run(`CTRL = 'lqr'; running = true; tick(1000);
      for (let i = 1; i <= ${fps*3}; i++) tick(1000 + i * 1000 / ${fps});
      ({t:T, x:Array.from(X)});`);
  });
  for (const end of ends) {
    assert.ok(Math.abs(end.t - 3) < 1e-9, `t=${end.t}`);
    end.x.forEach((value, i) => assert.ok(Math.abs(value - ends[0].x[i]) < 1e-9));
  }
});

test('LQR 표의 각 행이 현재 플랜트의 힘·모멘트 평형을 만족한다', () => {
  const sim=simulator();
  const residual=sim.run(`Math.max(...LQ.rows.map(r=>{
    const dx=xdot(r.x,r.u,[0,0,0],P0);
    return Math.hypot(...dx.slice(3,6),...dx.slice(10,13));
  }))`);
  assert.ok(residual<1e-6,`트림 잔차 ${residual}`);
});

test('배속 0.25~4에서도 40 ms마다 기록한다', () => {
  for (const speed of [0.25, 1, 4]) {
    const sim = simulator();
    const gaps = sim.run(`CTRL = 'lqr'; $('#rtf').value = '${speed}';
      running = true; tick(1000);
      for (let i = 1; i <= 60; i++) tick(1000 + i * 1000 / 60);
      REC.slice(1).map((r,i) => r.t - REC.at(i).t);`);
    assert.ok(gaps.length >= 6);
    gaps.forEach(gap => assert.ok(Math.abs(gap - 0.04) < 1e-9, `gap=${gap}`));
  }
});

test('시작·정지 상태를 즉시 그리고 초기화 시 별창에 빈 채점을 알린다', () => {
  const sim=simulator();
  const result=sim.run(`const painted=[];paint=()=>painted.push(running);
    toggleRun();toggleRun();
    const sent=[];scoreWin={closed:false,postMessage:value=>sent.push(value)};
    for(let i=0;i<500;i++)advanceStep();pushScore();
    reset();pushScore();({painted,old:sent[0].v,current:sent[1].v,rows:sent[1].s.t.length});`);
  assert.deepEqual(Array.from(result.painted),[true,false]);
  assert.ok(result.old);
  assert.equal(result.current,null);
  assert.equal(result.rows,1);
});

test('기본 LQR 비행과 고속 예제가 120초 후 속도·고도를 유지한다', () => {
  for (const target of [60, 83, 83.3]) {
    const sim = simulator();
    const result = sim.run(`$('#spd').value='${target}'; reset();
      for(let i=0;i<60000 && !physicsFailure;i++) advanceStep();
      ({score:score(),speed:diag().gs,z:X[2],t:T});`);
    assert.equal(result.t,120);
    assert.ok(Math.abs(result.speed-target)<0.1);
    assert.ok(Math.abs(result.z-200)<0.1);
    assert.equal(result.score.pass,true);
    assert.ok(result.score.maxOm<10);
  }
});

test('순항에서 감속해 호버로 복귀하며 각속도·포화 기준을 만족한다', () => {
  const sim=simulator();
  const result=sim.run(`$('#spd').value='60';reset();
    for(let i=0;i<60000;i++)advanceStep();$('#spd').value='0';
    for(let i=0;i<60000 && !physicsFailure;i++)advanceStep();
    ({s:score(),gs:diag().gs,z:X[2]});`);
  assert.ok(result.gs<0.1);
  assert.ok(Math.abs(result.z-200)<0.1);
  assert.equal(result.s.pass,true);
  assert.equal(result.s.satHi,0);
  assert.equal(result.s.satLo,0);
});

test('트림 앞먹임에서 전진비를 적용해 과도한 추력을 만들지 않는다', () => {
  const sim=simulator();
  const result=sim.run(`CTRL='trim';$('#spd').value='60';reset();
    for(let i=0;i<60000 && !physicsFailure;i++)advanceStep();
    ({s:score(),gs:diag().gs,z:X[2]});`);
  assert.ok(Math.abs(result.gs-60)<1);
  assert.ok(Math.abs(result.z-200)<1);
  assert.ok(result.s.maxOm<2);
  assert.equal(result.s.pass,true);
});

test('되감으면 풍속·계수·제어기까지 복원하고 같은 궤적으로 재개한다', () => {
  const sim=simulator();
  const result=sim.run(`
    for(let i=0;i<1000;i++) advanceStep();
    const expected=X.slice(), saved=REC.at(25);
    P.C_A0=0.7; $('#wsp').value='20'; CTRL='cascade';
    lastSatHi=true;lastSatLo=true;
    restoreTo(25); REC.truncate(26);
    const restored={coef:P.C_A0,wind:+$('#wsp').value,ctrl:CTRL,hi:lastSatHi,lo:lastSatLo};
    for(let i=0;i<500;i++) advanceStep();
    ({expected,actual:X,restored,rev:REC.at(-1).config.revision,originalRev:saved.config.revision});`);
  assert.equal(result.restored.coef,0.12);
  assert.equal(result.restored.wind,0);
  assert.equal(result.restored.ctrl,'lqr');
  assert.equal(result.restored.hi,false);
  assert.equal(result.restored.lo,false);
  assert.equal(result.rev,result.originalRev);
  result.expected.forEach((v,i)=>assert.ok(Math.abs(v-result.actual[i])<1e-9));
});

test('원형 기록은 최신 6,000개를 보관하고 이전 각속도 피크도 기억한다', () => {
  const sim=simulator();
  const result=sim.run(`
    health.maxOm=42;
    for(let i=1;i<=6010;i++){T=i*REC_EVERY; recordState();}
    ({n:REC.length,first:REC.at(0).t,last:REC.at(-1).t,s:score()});`);
  assert.equal(result.n,6000);
  assert.equal(result.first,0.44);
  assert.equal(result.last,240.4);
  assert.equal(result.s.tumble,true);
});

test('초기 지상 상태는 추락이 아니며, 이륙 후 접촉은 즉시 정지한다', () => {
  const sim=simulator();
  assert.equal(sim.run('ground();health.crashed'),false);
  const result=sim.run(`health.airborne=true;running=true;X[2]=-0.1;X[5]=-10;
    ground();({z:X[2],vz:X[5],crashed:health.crashed,running});`);
  assert.equal(result.z,0);
  assert.equal(result.vz,0);
  assert.equal(result.crashed,true);
  assert.equal(result.running,false);
});

test('NaN 제어 출력은 마지막 유효 상태를 보존하고 실패를 기록한다', () => {
  const sim=simulator();
  const result=sim.run(`for(let i=0;i<500;i++)advanceStep();
    const before=X.slice(); control=()=>[NaN,0,0,0];
    running=true;advanceStep();({before,after:X,diverged:health.diverged,running,
      failure:REC.at(-1).failure});`);
  assert.deepEqual(Array.from(result.after),Array.from(result.before));
  assert.equal(result.diverged,true);
  assert.equal(result.running,false);
  assert.match(result.failure,/NaN/);
});

test('CSV는 각 시점의 설정을 저장하며 되감은 뒤 미래 표본을 내보내지 않는다', () => {
  const sim=simulator();
  const csv=sim.run(`for(let i=0;i<40;i++) advanceStep();
    $('#wsp').value='10';P.C_A0=0.5;
    for(let i=0;i<40;i++) advanceStep();
    viewIdx=3;restoreTo(viewIdx);scoreCsv();`);
  const lines=csv.trim().split('\n').filter(l=>!l.startsWith('#'));
  const columns=lines.shift().split(',');
  const data=lines.map(l=>Object.fromEntries(l.split(',').map((v,i)=>[columns[i],v])));
  assert.equal(data.length,4);
  assert.equal(data[0].wind_mps,'0');
  assert.equal(data[3].wind_mps,'10');
  assert.equal(data[3].C_A0,'0.5');
  assert.equal(data[3].t_s,'0.12');
});

test('저속에서는 받음각을 정의하지 않고, 역류에서 모델 범위 경고를 낸다', () => {
  const sim=simulator();
  assert.equal(sim.run('diag().alpha'),null);
  const warnings=sim.run(`X[3]=-20;X[6]=0;X[7]=0;X[8]=0;X[9]=1;modelWarnings(diag());`);
  assert.ok(warnings.some(s=>s.includes('역류')));
});

test('채점은 미래 기록을 포함하지 않고 관찰이 짧으면 통과시키지 않는다', () => {
  const sim=simulator();
  const result=sim.run(`for(let i=0;i<1000;i++) advanceStep();
    const early=score();health.maxOm=50;recordState();
    viewIdx=30;restoreTo(viewIdx);({early,rewound:score()});`);
  assert.equal(result.early.pass,false);
  assert.equal(result.early.missionReady,false);
  assert.ok(result.rewound.maxOm<10);
  assert.equal(result.rewound.windowEnd,1.2);
});

test('단독 NMPC는 INDI/LQR 제어 없이 모터 명령을 직접 계산한다', () => {
  const sim=simulator();
  const result=sim.run(`CTRL='nmpc';$('#spd').value='0';reset();
    controlHybrid=()=>{throw Error('INDI 호출');};lqrLaw=()=>{throw Error('LQR 호출');};
    const u=control(X,cmdNow());({u,stat:motorStat});`);
  assert.equal(result.u.length,4);
  result.u.forEach(u=>assert.ok(Number.isFinite(u) && u>=0 && u<=1800));
  assert.ok(result.stat.cost<=result.stat.initialCost+1e-8);
});

test('단독 NMPC의 예측은 모터 지연·바람을 포함한 실제 플랜트와 일치한다', () => {
  const sim=simulator();
  const result=sim.run(`const [xt,ut]=lqrPick(60);const wind=[0,5,0];
    const predicted=motorPredict(xt,ut,wind,0.05);
    let reference=xt.slice();for(let i=0;i<25;i++)reference=rk4(reference,ut,wind,P,0.002);
    ({predicted,reference});`);
  result.reference.forEach((v,i)=>assert.ok(Math.abs(v-result.predicted[i])<0.002));
});

test('단독 NMPC의 웜스타트·출력을 되감기와 초기화에서 복원한다', () => {
  const sim=simulator();
  const result=sim.run(`CTRL='nmpc';$('#spd').value='0';reset();
    for(let i=0;i<600;i++)advanceStep();const expected=X.slice();
    restoreTo(27);REC.truncate(28);for(let i=0;i<60;i++)advanceStep();
    const actual=X.slice();reset();({expected,actual,cleared:motorU===null && motorOut===null});`);
  result.expected.forEach((v,i)=>assert.ok(Math.abs(v-result.actual[i])<1e-8));
  assert.equal(result.cleared,true);
});

test('단독 NMPC 압축 기울기가 실제 비용의 수치 미분과 일치한다', () => {
  const sim=simulator();
  // q·q_ref=0인 정확한 180도 경계는 부호 선택 비용의 미분 불가능점이므로 피한다.
  const errors=sim.run(`const [x]=lqrPick(30);x[2]=190;x[3]-=1;
    const ref=motorReference(x,{spd:30,alt:200});
    const U=Float64Array.from({length:4*MOTOR_MPC.N},(_,j)=>ref.u[j%4]/P.n_max);
    const previous=x.slice(13), rollout=motorRollout(x,U,ref,previous);
    const {g}=motorQuadratic(rollout.states,U,ref,previous);
    [0,1,10,25,47].map(j=>{
      const a=U.slice(),b=U.slice(),h=1e-6;a[j]+=h;b[j]-=h;
      const numeric=(motorRollout(x,a,ref,previous).cost-motorRollout(x,b,ref,previous).cost)/(2*h);
      return Math.abs(numeric-2*g[j])/Math.max(1,Math.abs(numeric));
    });`);
  errors.forEach(error=>assert.ok(error<0.001,`상대 오차 ${error}`));
});

test('단독 NMPC 풀이가 3회 연속 실패하면 숨은 제어기 전환 없이 정지한다', () => {
  const sim=simulator();
  const result=sim.run(`CTRL='nmpc';reset();motorSolve=()=>null;running=true;
    for(let i=0;i<600 && !physicsFailure;i++)advanceStep();
    ({running,ctrl:CTRL,failures:motorFailures,failure:physicsFailure,score:score()});`);
  assert.equal(result.running,false);
  assert.equal(result.ctrl,'nmpc');
  assert.equal(result.failures,3);
  assert.match(result.failure,/NMPC 풀이 연속 실패/);
  assert.equal(result.score.pass,false);
});

test('단독 NMPC는 호버·60·83 m/s 목표를 120초 후 유지한다', () => {
  for(const target of [0,60,83]){
    const sim=simulator();
    const result=sim.run(`CTRL='nmpc';$('#spd').value='${target}';reset();
      for(let i=0;i<60000 && !physicsFailure;i++)advanceStep();
      ({t:T,speed:diag().gs,z:X[2],score:score()});`);
    assert.equal(result.t,120);
    assert.ok(Math.abs(result.speed-target)<0.1);
    assert.ok(Math.abs(result.z-200)<0.1);
    assert.ok(result.score.maxOm<10);
    assert.equal(result.score.pass,true);
  }
});
