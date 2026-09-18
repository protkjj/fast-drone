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
    const g=mesh.selectedGeometry;
    const a=g?g.radial_arm_m/Math.SQRT2:.11938790893554688;
    const rotors=ROTOR_PARTS.map((part,index)=>{
      const count=part.ranges.reduce((sum,[first,end])=>sum+end-first,0);
      for(const [first,end]of part.ranges)owner.fill(index,first,end);
      return {center:[mesh.cgFromNose-(g?g.prop.plane_from_nose_m:.6317),part.yz[0]*a,part.yz[1]*a],
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

  function alignSelectedGeometry(mesh,g){
    // Explicit opt-in for the selected research aircraft. parse()/createVehicle()
    // without this transform retain the original STL used by the old 8 kg demo.
    if(mesh.triangles!==173804||g?.schema!=='selected-display-geometry-v1'||g.source_stl_sha256!==SOURCE_SHA256)
      throw new Error('Selected display requires the reviewed STL and geometry manifest');
    if(mesh.selectedGeometry)throw new Error('Selected display geometry is already aligned');
    if(!Number.isFinite(mesh.cgFromNose)||Math.abs(mesh.cgFromNose-g.cg_from_nose_m)>1e-10)
      throw new Error('Selected display CG does not match the mesh transform');
    const dimensions=[g.cg_from_nose_m,g.body_radius_m,g.radial_arm_m,
      ...['root_le_m','root_chord_m','tip_chord_m','span_m','leading_edge_sweep_m','thickness_m'].map(k=>g.fin?.[k]),
      ...['center_from_nose_m','diameter_m','length_m'].map(k=>g.pod?.[k]),
      ...['plane_from_nose_m','diameter_m'].map(k=>g.prop?.[k])];
    if(dimensions.some(v=>!Number.isFinite(v)||v<=0))
      throw new Error('Selected display dimensions must be finite and positive');
    const positions=mesh.positions.slice(),cg=mesh.cgFromNose;
    // Work in the source CAD coordinates: +x aft, then return to body +x nose.
    const cadPoint=i=>[cg-mesh.positions[i],mesh.positions[i+1],-mesh.positions[i+2]];
    const write=(i,v)=>{positions[i]=cg-v[0];positions[i+1]=v[1];positions[i+2]=-v[2];};
    for(const first of [8300,8312,8324,8336]){
      const points=[];
      for(let i=first*9;i<(first+12)*9;i+=3)points.push({i,v:cadPoint(i)});
      const sy=Math.sign(points[0].v[1]),sz=Math.sign(points[0].v[2]);
      for(const p of points){p.r=(sy*p.v[1]+sz*p.v[2])/Math.SQRT2;p.normal=(sy*p.v[1]-sz*p.v[2])/Math.SQRT2;}
      const low=Math.min(...points.map(p=>p.r)),high=Math.max(...points.map(p=>p.r));
      const root=points.filter(p=>Math.abs(p.r-low)<1e-7),tip=points.filter(p=>Math.abs(p.r-high)<1e-7);
      const rootLE=Math.min(...root.map(p=>p.v[0])),rootTE=Math.max(...root.map(p=>p.v[0]));
      const tipLE=Math.min(...tip.map(p=>p.v[0])),tipTE=Math.max(...tip.map(p=>p.v[0]));
      const halfThickness=Math.max(...points.map(p=>Math.abs(p.normal)));
      for(const p of points){
        const spanFraction=Math.min(1,Math.max(0,(p.r-low)/(high-low)));
        const oldLE=rootLE+(tipLE-rootLE)*spanFraction,oldTE=rootTE+(tipTE-rootTE)*spanFraction;
        const chordFraction=(p.v[0]-oldLE)/(oldTE-oldLE);
        const x=g.fin.root_le_m+g.fin.leading_edge_sweep_m*spanFraction+
          chordFraction*(g.fin.root_chord_m+(g.fin.tip_chord_m-g.fin.root_chord_m)*spanFraction);
        const r=g.body_radius_m+g.fin.span_m*spanFraction;
        const normal=p.normal/halfThickness*g.fin.thickness_m/2;
        write(p.i,[x,sy*(r+normal)/Math.SQRT2,sz*(r-normal)/Math.SQRT2]);
      }
    }
    // Relocate/resize each housing independently; do not scale the whole aircraft.
    for(const [first,end]of [[8348,10600],[10838,13090],[13328,15580],[15818,18070]]){
      const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
      for(let i=first*9;i<end*9;i+=3)cadPoint(i).forEach((v,k)=>{min[k]=Math.min(min[k],v);max[k]=Math.max(max[k],v);});
      const center=min.map((v,k)=>(v+max[k])/2),sy=Math.sign(center[1]),sz=Math.sign(center[2]);
      const oldAxis=[sy*.11938790893554688,sz*.11938790893554688];
      const radialScale=g.pod.diameter_m/Math.max(max[1]-min[1],max[2]-min[2]);
      for(let i=first*9;i<end*9;i+=3){const v=cadPoint(i);write(i,[
        g.pod.center_from_nose_m+(v[0]-center[0])*g.pod.length_m/(max[0]-min[0]),
        sy*g.radial_arm_m/Math.SQRT2+(v[1]-oldAxis[0])*radialScale,
        sz*g.radial_arm_m/Math.SQRT2+(v[2]-oldAxis[1])*radialScale]);}
    }
    const originalRotors=splitRotors(mesh).rotors;
    for(let index=0;index<4;index++){
      const rotor=originalRotors[index],part=ROTOR_PARTS[index];
      const center=[cg-g.prop.plane_from_nose_m,part.yz[0]*g.radial_arm_m/Math.SQRT2,
        part.yz[1]*g.radial_arm_m/Math.SQRT2];
      const scale=g.prop.diameter_m/(2*rotor.radius);
      for(const [first,end]of part.ranges)for(let i=first*9;i<end*9;i+=3){
        positions[i]=center[0]+mesh.positions[i]-rotor.center[0];
        positions[i+1]=center[1]+(mesh.positions[i+1]-rotor.center[1])*scale;
        positions[i+2]=center[2]+(mesh.positions[i+2]-rotor.center[2])*scale;
      }
    }
    const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
    for(let i=0;i<positions.length;i++){const k=i%3;min[k]=Math.min(min[k],positions[i]);max[k]=Math.max(max[k],positions[i]);}
    return {...mesh,positions,min,max,selectedGeometry:JSON.parse(JSON.stringify(g))};
  }

  function createVehicle(mesh,THREE){
    const parts=splitRotors(mesh),vehicle=new THREE.Group();vehicle.name='drone_v2.stl';
    vehicle.userData.geometryBasis=mesh.selectedGeometry?'selected_design.csv':'original drone_v2.stl';
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
  const api={parse,splitRotors,alignSelectedGeometry,createVehicle,animateRotors,SOURCE_SHA256};
  if(typeof module!=='undefined'&&module.exports) module.exports=api;
  else root.ResearchSTL=api;
})(globalThis);
