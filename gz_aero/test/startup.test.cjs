const test = require('node:test');
const assert = require('node:assert/strict');
const {simulator} = require('./sim_harness.cjs');

test('지상 초기조건은 실제 모터 0이며 호버 기준 데이터는 보존한다', () => {
  const s = simulator();
  const r = s.run(`({z:X[2],v:X.slice(3,6),motors:X.slice(13),reference:D.x0.slice(13),
    phase:REC.at(0).phase});`);
  assert.equal(r.z,0);
  assert.deepEqual(Array.from(r.v),[0,0,0]);
  assert.deepEqual(Array.from(r.motors),[0,0,0,0]);
  assert.ok(r.reference.every(n=>n>0));
  assert.equal(r.phase,'stopped');
});

test('시동 중 제어기는 누적되지 않고 모터 ODE만 공통 공회전 명령을 따른다', () => {
  const s = simulator();
  const r = s.run(`control=()=>{throw Error('시동 중 비행 제어기 호출');};
    const samples=[];
    for(let i=0;i<500;i++){advanceStep();samples.push({z:X[2],vz:X[5],n:X[13],cmd:cmdSpd});}
    ({samples,idle:startupIdleSpeed(),phase:REC.at(-1).phase,altI});`);
  assert.ok(r.samples[0].n>0 && r.samples[0].n<r.idle/100);
  r.samples.forEach((v,i)=>{
    assert.equal(v.z,0);assert.equal(v.vz,0);assert.equal(v.cmd,0);
    assert.ok(v.n<r.idle);
    if(i)assert.ok(v.n>=r.samples[i-1].n);
  });
  assert.equal(r.phase,'takeoff');assert.equal(r.altI,0);
});

test('지면 반력은 무게를 지지하며 모터 응답을 순간 변경하지 않는다', () => {
  const s = simulator();
  const r = s.run(`const n=0.5*D.n_hover,u=Array(4).fill(n),x=X.slice();
    const supported=contactXdot(x,u,[0,0,0],P);
    const next=rk4(x,u,[0,0,0],P,0.1,200,true);
    ({supported,next,expected:n*(1-Math.exp(-0.1/P.tau_m))});`);
  assert.equal(r.supported[5],0);
  assert.equal(r.next[2],0);assert.equal(r.next[5],0);
  assert.ok(Math.abs(r.next[13]-r.expected)<1e-6);
});

test('실제 상향 힘이 무게를 넘은 뒤에만 이륙하고 0 명령이면 계속 지상에 남는다', () => {
  const s = simulator();
  const r = s.run(`const rows=[];
    for(let i=0;i<1000 && !health.airborne;i++){
      advanceStep();rows.push({z:X[2],az:xdot(X,X.slice(13),windNow(),P)[5]});
    }
    const lifted=health.airborne, last=rows.at(-1), liftTime=T;
    reset();control=()=>[0,0,0,0];
    for(let i=0;i<2000;i++)advanceStep();
    ({lifted,last,liftTime,z:X[2],vz:X[5],airborne:health.airborne});`);
  assert.equal(r.lifted,true);assert.ok(r.liftTime>1);
  assert.ok(r.last.z>0 && r.last.az>0);
  assert.equal(r.z,0);assert.equal(r.vz,0);assert.equal(r.airborne,false);
});

test('시동 되감기는 모터·회전각·명령·접촉 상태를 같은 값으로 재현한다', () => {
  const s = simulator();
  const r = s.run(`for(let i=0;i<700;i++)advanceStep();
    const expected={x:X.slice(),phase:rotorPhase.slice(),cmd:cmdSpd,record:REC.at(-1).phase};
    restoreTo(10);REC.truncate(11);for(let i=0;i<500;i++)advanceStep();
    ({expected,actual:{x:X.slice(),phase:rotorPhase.slice(),cmd:cmdSpd,record:REC.at(-1).phase}});`);
  assert.deepEqual(JSON.parse(JSON.stringify(r.actual)),JSON.parse(JSON.stringify(r.expected)));
});

test('지상 접촉은 공중 운동식과 분리되어 자유비행 회귀 기준을 바꾸지 않는다', () => {
  const s = simulator();
  const r = s.run(`const [x,u]=lqrPick(30);x[2]=20;
    ({free:rk4(x,u,[0,0,0],P,DT),contact:rk4(x,u,[0,0,0],P,DT,4,true),
      error:selfCheck()});`);
  assert.deepEqual(Array.from(r.free),Array.from(r.contact));
  assert.ok(r.error<1e-9);
});

test('CSV는 정지·시동·비행과 명령/실제 회전수·지면 반력을 구분한다', () => {
  const s = simulator();
  const csv=s.run(`for(let i=0;i<700;i++)advanceStep();scoreCsv();`);
  const lines=csv.trim().split('\n').filter(line=>!line.startsWith('#'));
  const keys=lines.shift().split(',');
  const rows=lines.map(line=>Object.fromEntries(line.split(',').map((v,i)=>[keys[i],v])));
  assert.equal(rows[0].startup_phase,'stopped');
  assert.equal(+rows[0].motor_1_actual_rad_s,0);
  assert.equal(+rows[0].motor_1_command_rad_s,0);
  assert.ok(Math.abs(+rows[0].ground_normal_N-78.48)<1e-6);
  const spool=rows.find(r=>r.startup_phase==='spoolup');
  assert.ok(+spool.motor_1_command_rad_s>+spool.motor_1_actual_rad_s);
  assert.ok(+spool.ground_normal_N>0);
  const flight=rows.find(r=>r.startup_phase==='flight');
  assert.ok(+flight.alt_m>0);assert.equal(+flight.ground_normal_N,0);
});
