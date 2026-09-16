/* Binary STL parser and a proper (non-mirroring) CAD-to-body transform. */
(function(root){
  'use strict';
  // Face ranges audited by inspect_stl.py (21 disconnected CAD components).
  // Order follows body (y,z): (+,+),(-,+),(-,-),(+,-), just like the plant.
  const SOURCE_SHA256='276045699ee8e9ee03fe5d75d4915d88a0688fbcb12a9e8d1e8f5c0702302e35';
  const ROTOR_PARTS=[
    {yz:[1,1],ranges:[[18070,18308],[132560,173804]]},
    {yz:[-1,1],ranges:[[15580,15818],[96056,132560]]},
    {yz:[-1,-1],ranges:[[13090,13328],[54812,96056]]},
    {yz:[1,-1],ranges:[[10600,10838],[18308,54812]]}
  ];
  function parse(buffer,cgFromNose=.4282511278030152) {
    if(buffer.byteLength<84) throw new Error('STL header is incomplete');
    const view=new DataView(buffer), triangles=view.getUint32(80,true);
    if(triangles>500000||buffer.byteLength!==84+50*triangles) throw new Error('Unexpected binary STL length');
    const positions=new Float32Array(triangles*9);
    const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
    for(let triangle=0;triangle<triangles;triangle++) for(let vertex=0;vertex<3;vertex++) {
      const offset=84+50*triangle+12+12*vertex;
      // CAD: mm, x=0 nose, x positive aft. Rotate 180 deg about Y, then
      // translate by CG. This preserves handedness (unlike flipping X alone).
      const coordinates=[cgFromNose-view.getFloat32(offset,true)*.001,
                          view.getFloat32(offset+4,true)*.001,-view.getFloat32(offset+8,true)*.001];
      coordinates.forEach((v,i)=>{
        if(!Number.isFinite(v)) throw new Error('Non-finite STL coordinate');
        positions[triangle*9+vertex*3+i]=v; min[i]=Math.min(min[i],v);max[i]=Math.max(max[i],v);
      });
    }
    return {positions,triangles,min,max,cgFromNose};
  }

  function splitRotors(mesh){
    if(mesh.triangles!==173804)throw new Error('Rotor partition requires the reviewed drone_v2.stl');
    const owner=new Int8Array(mesh.triangles);owner.fill(-1);
    const a=.11938790893554688;
    const rotors=ROTOR_PARTS.map((part,index)=>{
      const count=part.ranges.reduce((sum,[first,end])=>sum+end-first,0);
      for(const [first,end]of part.ranges)owner.fill(index,first,end);
      return {center:[mesh.cgFromNose-.6317,part.yz[0]*a,part.yz[1]*a],
        positions:new Float32Array(count*9),ranges:part.ranges,offset:0,radius:0};
    });
    const body=new Float32Array(owner.reduce((count,index)=>count+(index<0?1:0),0)*9);
    let bodyOffset=0;
    for(let face=0;face<mesh.triangles;face++){
      const rotor=rotors[owner[face]];
      for(let coordinate=0;coordinate<9;coordinate++){
        const value=mesh.positions[face*9+coordinate];
        if(rotor)rotor.positions[rotor.offset++]=value-rotor.center[coordinate%3];
        else body[bodyOffset++]=value;
      }
    }
    for(const rotor of rotors){
      for(let i=0;i<rotor.positions.length;i+=3)
        rotor.radius=Math.max(rotor.radius,Math.hypot(rotor.positions[i+1],rotor.positions[i+2]));
      delete rotor.offset;
    }
    return {body,rotors};
  }

  function createVehicle(mesh,THREE){
    const parts=splitRotors(mesh),vehicle=new THREE.Group();vehicle.name='drone_v2.stl';
    const geometry=positions=>{
      const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(positions,3));
      g.computeVertexNormals();return g;
    };
    const skin=new THREE.MeshStandardMaterial({color:0xb5c8d5,metalness:.22,roughness:.55,side:THREE.DoubleSide});
    const body=new THREE.Mesh(geometry(parts.body),skin);body.name='airframe';vehicle.add(body);
    vehicle.userData.rotors=parts.rotors.map((part,index)=>{
      const rotor=new THREE.Group();rotor.name='rotor-'+(index+1);rotor.position.set(...part.center);
      const material=new THREE.MeshStandardMaterial({color:0x9cb7c8,metalness:.25,roughness:.5,
        side:THREE.DoubleSide,transparent:true,opacity:1});
      const blades=new THREE.Mesh(geometry(part.positions),material);rotor.add(blades);
      const blur=new THREE.Mesh(new THREE.RingGeometry(.014,part.radius,64),
        new THREE.MeshBasicMaterial({color:0xa7d4e8,transparent:true,opacity:.14,
          side:THREE.DoubleSide,depthWrite:false}));
      blur.rotation.y=Math.PI/2;blur.visible=false;rotor.add(blur);
      rotor.userData={blades,blur};vehicle.add(rotor);return rotor;
    });
    return vehicle;
  }

  function animateRotors(vehicle,rates,dt,active,directions=[1,-1,1,-1]){
    // Deliberately illustrative: cap visual angular speed to avoid frame-rate
    // aliasing. Never overwrite the actual RPM, rotor phase, forces or moments.
    const elapsed=active?Math.min(.05,Math.max(0,dt)):0;
    (vehicle.userData.rotors||[]).forEach((rotor,index)=>{
      const rate=Number.isFinite(rates[index])?Math.max(0,rates[index]):0;
      rotor.rotation.x=(rotor.rotation.x-directions[index]*18*Math.tanh(rate/250)*elapsed)%(2*Math.PI);
      const strength=active?Math.min(1,rate/600):0;
      rotor.userData.blur.visible=strength>.02;
      rotor.userData.blur.material.opacity=.14*strength;
      rotor.userData.blades.material.opacity=1-.25*strength;
    });
  }
  const api={parse,splitRotors,createVehicle,animateRotors,SOURCE_SHA256};
  if(typeof module!=='undefined'&&module.exports) module.exports=api;
  else root.ResearchSTL=api;
})(globalThis);
