const test = require('node:test');
const assert = require('node:assert/strict');
const {simulator} = require('./sim_harness.cjs');

test('Hybrid: 추력 0과 불가능한 회전 명령을 함께 줘도 추력을 만들어내지 않는다', () => {
  const s=simulator();
  const r=s.run(`const [x]=lqrPick(83); resetHybridInterface();
    const u=hybridIndi(x,[0,25,25,25],DT);
    ({u,stat:hybridStat,actual:u.map(n=>hybridRotorForce(n,axialInflow(x)))});`);
  assert.ok(Math.abs(r.actual.reduce((a,b)=>a+b,0))<1e-7);
  assert.equal(r.stat.requested[0],0);
  assert.ok(r.stat.measured[0]>1,'실제 모터 추력이 순간적으로 0으로 바뀌면 안 됨');
  assert.ok(Math.hypot(...r.stat.residual.slice(1))>1);
});

test('Hybrid: 실현 가능한 증분 명령은 총추력과 세 각가속도를 함께 만족한다', () => {
  const s=simulator();
  const r=s.run(`const B=hybridForceMatrix(); const desired=[20,25,22,27];
    const angular=B.map(row=>row.reduce((sum,b,i)=>sum+b*desired[i],0));
    const f=hybridAllocate(94,angular,B,[200,200,200,200]);({f,desired});`);
  r.desired.forEach((f,i)=>assert.ok(Math.abs(f-r.f[i])<1e-7));
});

test('Hybrid: 포화 할당은 각 로터 경계와 총추력 등식을 지킨다', () => {
  const s=simulator();
  const results=s.run(`Array.from({length:100},(_,k)=>{
    const caps=[10,20,30,40], total=k*1.3;
    const target=[k-30,100*Math.sin(k),100*Math.cos(k)],B=hybridForceMatrix();
    const f=hybridAllocate(total,target,B,caps);
    const residual=B.map((row,k)=>row.reduce((sum,b,i)=>sum+b*f[i],0)-target[k]);
    const gradient=f.map((_,i)=>B.reduce((sum,row,k)=>sum+row[i]*residual[k],0));
    let descent=0;
    for(let i=0;i<4;i++)for(let j=0;j<4;j++){
      if(f[i]<caps[i]-1e-6 && f[j]>1e-6)descent=Math.min(descent,gradient[i]-gradient[j]);
    }
    return {f,total:Math.min(total,100),descent};
  });`);
  for(const r of results){
    r.f.forEach((f,i)=>assert.ok(f>=-1e-8 && f<=[10,20,30,40][i]+1e-8));
    assert.ok(Math.abs(r.f.reduce((a,b)=>a+b,0)-r.total)<1e-7);
    assert.ok(r.descent>-1e-7,'총추력을 보존하는 비용 감소 방향이 남으면 안 됨');
  }
});

test('Hybrid: 실제 로터 상태를 읽고, 추력 피드백과 재생 상태를 보존한다', () => {
  const s=simulator();
  const r=s.run(`const [x]=lqrPick(60);resetHybridInterface();
    hybridIndi(x,[80,0,0,0],DT);const saved=hybridSnapshot();
    const a=hybridIndi(x,[90,1,2,3],DT);resetHybridInterface();hybridRestore(saved);
    const b=hybridIndi(x,[90,1,2,3],DT);
    ({a,b,stat:hybridStat,T:x.slice(13).reduce((sum,n)=>sum+hybridRotorForce(n,axialInflow(x)),0)});`);
  assert.deepEqual(Array.from(r.a),Array.from(r.b));
  assert.ok(Math.abs(r.T-r.stat.measured[0])<1e-9);
});

test('Hybrid: 힘 좌표 효과행렬이 +x 플랜트의 각가속도 변화와 일치한다', () => {
  const s=simulator();
  const errors=s.run(`const [x]=lqrPick(60);x[10]=x[11]=x[12]=0;
    const B=hybridForceMatrix(),vax=axialInflow(x),base=xdot(x,x.slice(13),[0,0,0],P);
    Array.from({length:4},(_,i)=>{
      const xp=x.slice(),f=hybridRotorForce(x[13+i],vax);xp[13+i]=nFromThrust(f+0.01,vax);
      const changed=xdot(xp,xp.slice(13),[0,0,0],P);
      return Math.max(...B.map((row,k)=>Math.abs((changed[10+k]-base[10+k])/0.01-row[i])));
    });`);
  errors.forEach(e=>assert.ok(e<1e-5));
});

test('Hybrid: NMPC가 정한 네 명령을 덮어쓰지 않고 할당 결과를 되먹인다', () => {
  const s=simulator();
  const r=s.run(`CTRL='sqprti';reset(); const command=[78.48,1,2,3];
    rtiSolve=()=>command.slice();controlHybrid(X,{spd:0,alt:0,psi:0});
    ({requested:hybridStat.requested,applied:hybridStat.applied,last:rtiApplied});`);
  assert.deepEqual(Array.from(r.requested),[78.48,1,2,3]);
  assert.deepEqual(Array.from(r.last),Array.from(r.applied));
});

test('Hybrid: 가상 모델의 병진 가속도는 바람을 포함한 플랜트와 일치한다', () => {
  const s=simulator();
  const r=s.run(`$('#wsp').value='10';$('#wdir').value='90';
    const [x]=lqrPick(60),vax=axialInflow(x);
    const total=x.slice(13).reduce((sum,n)=>sum+hybridRotorForce(n,vax),0);
    ({virtual:vXdot(x,[total,0,0,0],P).slice(3,6),
      physical:xdot(x,x.slice(13),windNow(),P).slice(3,6)});`);
  r.virtual.forEach((v,i)=>assert.ok(Math.abs(v-r.physical[i])<1e-10));
});

test('Hybrid: 비정상 NMPC 출력은 유효 상태를 보존하고 정지한다', () => {
  const s=simulator();
  const r=s.run(`CTRL='sqprti';reset();for(let i=0;i<500;i++)advanceStep();const before=X.slice();
    rtiSolve=()=>[NaN,0,0,0];running=true;advanceStep();
    ({before,after:X,running,failed:health.diverged});`);
  assert.deepEqual(Array.from(r.before),Array.from(r.after));
  assert.equal(r.running,false);assert.equal(r.failed,true);
});

test('Hybrid: CSV는 당시의 요청·할당·측정과 샘플 시각을 구분해 기록한다', () => {
  const s=simulator();
  const csv=s.run(`CTRL='sqprti';reset();for(let i=0;i<540;i++)advanceStep();scoreCsv();`);
  const rows=csv.trim().split('\n').filter(row=>!row.startsWith('#'));
  const columns=rows.shift().split(',');
  const records=rows.map(row=>Object.fromEntries(row.split(',').map((v,i)=>[columns[i],v])));
  assert.equal(records[0].hybrid_request_T_N,'');
  assert.equal(records[1].hybrid_sample_t_s,'');
  assert.equal(records[1].startup_phase,'spoolup');
  assert.equal(records[26].hybrid_sample_t_s,'1.038');
  assert.equal(records[26].t_s,'1.04');
  for(const prefix of ['request','allocated','measured','residual']){
    assert.ok(records[26]['hybrid_'+prefix+'_T_N']!=='');
    assert.ok(Number.isFinite(+records[26]['hybrid_'+prefix+'_T_N']));
  }
});

test('Hybrid: 되감기 후 필터·할당·NMPC 상태까지 같은 궤적을 재현한다', () => {
  const s=simulator();
  const r=s.run(`CTRL='sqprti';reset();for(let i=0;i<1500;i++)advanceStep();
    const expected=X.slice();restoreTo(35);REC.truncate(36);
    for(let i=0;i<800;i++)advanceStep();const actual=X.slice();reset();
    ({expected,actual,cleared:hybridState===null && hybridStat===null});`);
  r.expected.forEach((v,i)=>assert.ok(Math.abs(v-r.actual[i])<1e-8));
  assert.equal(r.cleared,true);
});

test('Hybrid: SQP-RTI는 호버·60·83 m/s에서 120초 비행 기준을 만족한다', () => {
  for(const target of [0,60,83]){
    const s=simulator();
    const r=s.run(`CTRL='sqprti';$('#spd').value='${target}';reset();
      let thrustError=0;
      for(let i=0;i<60000 && !physicsFailure;i++){
        advanceStep();
        if(T>30)thrustError=Math.max(thrustError,Math.abs(hybridStat.residual[0]));
      }
      ({t:T,gs:diag().gs,z:X[2],s:score(),thrustError});`);
    assert.equal(r.t,120);assert.equal(r.s.pass,true,JSON.stringify(r));
    assert.ok(Math.abs(r.gs-target)<0.1);assert.ok(Math.abs(r.z-200)<0.1);
    assert.ok(r.thrustError<1e-6);
  }
});
