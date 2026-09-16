const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const {createHash}=require('node:crypto');
const stl=require('../stl.js');

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
