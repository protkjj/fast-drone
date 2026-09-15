/* Binary STL parser and a proper (non-mirroring) CAD-to-body transform. */
(function(root){
  'use strict';
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
    return {positions,triangles,min,max};
  }
  if(typeof module!=='undefined'&&module.exports) module.exports={parse};
  else root.ResearchSTL={parse};
})(globalThis);
