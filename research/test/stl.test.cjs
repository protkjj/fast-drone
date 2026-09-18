const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const {createHash}=require('node:crypto');
const stl=require('../stl.js');
const selectedGeometry=require('../assets/selected_geometry.json');

test('CAD rotor partition preserves every original triangle at zero phase',()=>{
  const bytes=fs.readFileSync(path.join(__dirname,'../assets/drone_v2.stl'));
  assert.equal(createHash('sha256').update(bytes).digest('hex'),stl.SOURCE_SHA256);
  const mesh=stl.parse(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
  const parts=stl.splitRotors(mesh);
  assert.equal(parts.body.length/9,17356);assert.equal(parts.rotors.length,4);
  assert.equal(parts.body.length+parts.rotors.reduce((sum,r)=>sum+r.positions.length,0),mesh.positions.length);
  const claimed=new Set();
  for(const rotor of parts.rotors){
    assert.ok(rotor.radius>.065&&rotor.radius<.069);
    let offset=0;
    for(const [first,end]of rotor.ranges)for(let face=first;face<end;face++){
      assert.ok(!claimed.has(face));claimed.add(face);
      for(let coordinate=0;coordinate<9;coordinate++){
        const restored=rotor.positions[offset++]+rotor.center[coordinate%3];
        assert.ok(Math.abs(restored-mesh.positions[face*9+coordinate])<3e-8);
      }
    }
  }
});

test('illustrative rotor motion never changes RPM input and freezes when paused',()=>{
  const rates=Object.freeze([1500,1500,0,1500]);
  const rotors=Array.from({length:4},()=>({rotation:{x:0},userData:{
    blades:{material:{opacity:1}},blur:{visible:false,material:{opacity:0}}}}));
  const vehicle={userData:{rotors}};
  stl.animateRotors(vehicle,rates,.02,true);
  assert.ok(rotors[0].rotation.x<0);assert.ok(rotors[1].rotation.x>0);
  assert.equal(rotors[2].rotation.x,0);assert.equal(rotors[2].userData.blur.visible,false);
  const before=rotors.map(r=>r.rotation.x);
  stl.animateRotors(vehicle,rates,100,false);
  assert.deepEqual(rotors.map(r=>r.rotation.x),before);
  assert.deepEqual(rates,[1500,1500,0,1500]);
  assert.ok(rotors.every(r=>!r.userData.blur.visible&&r.userData.blades.material.opacity===1));
});

test('CSV-based display changes fin planform and rotor placement without mutating the original mesh',()=>{
  const bytes=fs.readFileSync(path.join(__dirname,'../assets/drone_v2.stl'));
  const original=stl.parse(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
  const before=original.positions.slice();
  const aligned=stl.alignSelectedGeometry(original,selectedGeometry);
  assert.deepEqual(original.positions,before);
  assert.notEqual(aligned.positions,original.positions);
  assert.equal(aligned.triangles,original.triangles);
  assert.deepEqual(aligned.positions.slice(0,8300*9),original.positions.slice(0,8300*9)); // body skin unchanged
  const p=selectedGeometry,fin=p.fin;
  const close=(a,b)=>assert.ok(Math.abs(a-b)<1e-7,`${a} != ${b}`);
  for(const first of [8300,8312,8324,8336]){
    const sy=Math.sign(aligned.positions[first*9+1]),sz=Math.sign(-aligned.positions[first*9+2]);
    const points=[];
    for(let i=first*9;i<(first+12)*9;i+=3){
      const x=p.cg_from_nose_m-aligned.positions[i],y=aligned.positions[i+1],z=-aligned.positions[i+2];
      points.push([x,(sy*y+sz*z)/Math.SQRT2,(sy*y-sz*z)/Math.SQRT2]);
    }
    const root=points.filter(v=>Math.abs(v[1]-p.body_radius_m)<1e-7);
    const tip=points.filter(v=>Math.abs(v[1]-p.body_radius_m-fin.span_m)<1e-7);
    assert.ok(root.length&&tip.length);
    close(Math.min(...root.map(v=>v[0])),fin.root_le_m);
    close(Math.max(...root.map(v=>v[0])),fin.root_le_m+fin.root_chord_m);
    close(Math.min(...tip.map(v=>v[0])),fin.root_le_m+fin.leading_edge_sweep_m);
    close(Math.max(...tip.map(v=>v[0])),fin.root_le_m+fin.leading_edge_sweep_m+fin.tip_chord_m);
    close(Math.max(...points.map(v=>v[2]))-Math.min(...points.map(v=>v[2])),fin.thickness_m);
  }
  for(const [first,end]of [[8348,10600],[10838,13090],[13328,15580],[15818,18070]]){
    const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
    for(let i=first*9;i<end*9;i+=3)for(let k=0;k<3;k++){
      min[k]=Math.min(min[k],aligned.positions[i+k]);max[k]=Math.max(max[k],aligned.positions[i+k]);
    }
    close((min[0]+max[0])/2,p.cg_from_nose_m-p.pod.center_from_nose_m);
    close(max[0]-min[0],p.pod.length_m);
    close(Math.max(max[1]-min[1],max[2]-min[2]),p.pod.diameter_m);
  }
  for(const rotor of stl.splitRotors(aligned).rotors){
    close(rotor.center[0],p.cg_from_nose_m-p.prop.plane_from_nose_m);
    close(Math.hypot(rotor.center[1],rotor.center[2]),p.radial_arm_m);
    close(rotor.radius,p.prop.diameter_m/2);
  }
  assert.throws(()=>stl.alignSelectedGeometry(aligned,selectedGeometry),/already aligned/);
});

test('display geometry is traceable to the audited CSV source and is not another physical model',()=>{
  const report=require('../validation-design-source-2026-09-18.json'),p=require('../profiles/selected.json');
  assert.equal(selectedGeometry.source_csv_sha256,p.provenance.sha256);
  assert.equal(selectedGeometry.source_commit,p.provenance.commit);
  assert.equal(selectedGeometry.source_stl_sha256,stl.SOURCE_SHA256);
  for(const [key,value]of Object.entries(selectedGeometry.fin))assert.equal(value,report.source_fin_geometry[key]);
  assert.equal(selectedGeometry.radial_arm_m,p.arm_m);
  assert.equal(selectedGeometry.cg_from_nose_m,p.cg_from_nose_m);
  assert.equal(selectedGeometry.prop.plane_from_nose_m,report.source_prop_location_from_nose_m);
  assert.equal(selectedGeometry.prop.diameter_m,p.prop.diameter_m);
});

test('CSV display transform rejects mismatched source metadata or malformed dimensions',()=>{
  assert.throws(()=>stl.alignSelectedGeometry({triangles:0},selectedGeometry),/reviewed/);
  assert.throws(()=>stl.alignSelectedGeometry({triangles:173804,cgFromNose:0},selectedGeometry),/CG/);
  const invalid=structuredClone(selectedGeometry);invalid.fin.thickness_m=0;
  assert.throws(()=>stl.alignSelectedGeometry({triangles:173804,cgFromNose:selectedGeometry.cg_from_nose_m},invalid),/dimensions/);
  const missing=structuredClone(selectedGeometry);delete missing.fin.span_m;missing.fin.wrong_span_m=.12;
  assert.throws(()=>stl.alignSelectedGeometry({triangles:173804,cgFromNose:selectedGeometry.cg_from_nose_m},missing),/dimensions/);
});
