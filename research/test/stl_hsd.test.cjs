const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const {createHash}=require('node:crypto');
const stl=require('../stl.js');
const profile=require('../profiles/selected.json');

// 형상팀 HSD_drone_assembly.step 기반 표시 메시. drone_v2.stl(선정 CSV와 형상이
// 달라 alignSelectedGeometry로 맞춤 변형이 필요했음)과 달리, 이 CAD는 동체
// 길이·지름·암 반경이 selected.json 값과 이미 mm 단위로 일치해 별도 변형 없이
// 그대로 쓴다 (research/DESIGN_SOURCE_AUDIT.md의 provenance 방침 참고).
test('HSD CAD mesh matches the reviewed hash and preserves every triangle across body/rotor split',()=>{
  const bytes=fs.readFileSync(path.join(__dirname,'../assets/drone_hsd.stl'));
  assert.equal(createHash('sha256').update(bytes).digest('hex'),stl.HSD_SOURCE_SHA256);
  const mesh=stl.parse(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength),profile.cg_from_nose_m);
  assert.equal(mesh.triangles,211220);
  const parts=stl.splitRotors(mesh);
  assert.equal(parts.rotors.length,4);
  assert.equal(parts.body.length+parts.rotors.reduce((sum,r)=>sum+r.positions.length,0),mesh.positions.length);
  const claimed=new Set();
  for(const rotor of parts.rotors){
    // 4-fold 대칭 CAD라 반경이 사실상 동일해야 한다 -- 라벨링 버그(예: z부호
    // 누락)가 있으면 로터마다 다른, 훨씬 큰 반경이 나온다(직접 겪은 회귀).
    assert.ok(rotor.radius>.06&&rotor.radius<.075,`unexpected rotor radius ${rotor.radius}`);
    for(const [first,end]of rotor.ranges)for(let face=first;face<end;face++){
      assert.ok(!claimed.has(face));claimed.add(face);
    }
  }
  assert.equal(claimed.size,211220-parts.body.length/9);
});

test('HSD mesh key dimensions match the CSV-pinned selected profile to sub-millimetre precision',()=>{
  const bytes=fs.readFileSync(path.join(__dirname,'../assets/drone_hsd.stl'));
  const mesh=stl.parse(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength),profile.cg_from_nose_m);
  const bodyLength=mesh.max[0]-mesh.min[0];
  assert.ok(Math.abs(bodyLength-profile.body_length_m)<2e-4,`body length ${bodyLength} vs ${profile.body_length_m}`);
  const parts=stl.splitRotors(mesh);
  for(const rotor of parts.rotors){
    const armRadius=Math.hypot(rotor.center[1],rotor.center[2]);
    assert.ok(Math.abs(armRadius-profile.arm_m)<1e-4,`arm radius ${armRadius} vs ${profile.arm_m}`);
  }
});

test('HSD mesh source is not silently swappable for an unreviewed file',()=>{
  assert.throws(()=>stl.splitRotors({triangles:12345}),/reviewed mesh/);
});
