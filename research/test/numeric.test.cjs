const {test,before}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),R=require('../runtime.js');
let ca;
before(async()=>{ca=await require('@casadi/casadi-wasm')();await ca.load_interpolant('linear');});
function close(actual,expected,label){
  assert.equal(actual.length,expected.length,label);
  actual.forEach((row,i)=>{assert.equal(row.length,expected[i].length,label);row.forEach((v,j)=>{
    const ref=expected[i][j];assert.ok(Number.isFinite(v)&&Math.abs(v-ref)<=2e-9*Math.max(1,Math.abs(ref)),`${label}[${i}][${j}]: ${v} vs ${ref}`);
  });});
}
for(const profile of ['simple','selected'])test(`${profile}: generated scalar graphs match original WASM including sparse outputs and map boundaries`,()=>{
  const data=JSON.parse(fs.readFileSync(path.join(__dirname,`../generated/${profile}.json`)));
  assert.equal(data.numeric.format,'casadi-scalar-js-v1');
  const fast=R.makeBindings(ca,data,[],true),original=R.makeBindings(ca,data,[],false),noise=R.random(8128);
  function check(group,key,args){close(fast.call(fast[group][key],...args),original.call(original[group][key],...args),key);}
  try{
    for(let i=0;i<36;i++){
      const x=data.initial.slice(),env=[noise()*4,noise()*4,noise()*4,...[0,0,0]],scales=[1,1,1,1,1,1].map(()=>1+noise()*.08);
      x[3]=noise()*35;x[4]=noise()*8;x[5]=noise()*8;
      const q=Array.from({length:4},noise),norm=Math.hypot(...q);q.forEach((v,j)=>x[6+j]=v/norm);
      for(let j=10;j<13;j++)x[j]=noise();
      for(let j=13;j<17;j++)x[j]=i===0?0:Math.max(0,1500+noise()*500);
      x[17]=[0,.5,1,.2][i%4];
      const u=x.slice(13,17).map(v=>Math.max(0,v+noise()*700));
      for(const key of ['rhs','step','diag'])check('functions',key,[x,u,env,scales]);
      check('functions','aero',[x.slice(0,13),env.slice(0,3)]);
      const axial=i%2===0?0:noise()*40;
      check('functions','rotors',[x.slice(13,17),[axial]]);
      check('functions','motor_step',[x.slice(13,17),u,[axial],[22]]);
      check('functions','inverse_thrust',[[0,2,5,20],[axial],[3000]]);
    }
    // Every interpolation knot, including derivatives at the boundary.
    for(const j of data.profile.prop.J||[0,.5,1]){
      const n=1500,axial=j*n/(2*Math.PI)*data.profile.prop.diameter_m;
      check('functions','rotors',[[n,n,n,n],[axial]]);
    }
    const est=new R.Estimator(original,data,data.initial);
    check('estimator','predict',[est.state,est.cov,[9.8,.1,-.2,.03,-.04,.02]]);
    check('estimator','gps',[est.state,est.cov,[.1,-.1,20.2,.2,0,-.1]]);
  }finally{fast.dispose();original.dispose();}
});
test('generated and original equations agree through a closed-loop sensor/INDI run',async()=>{
  const data=JSON.parse(fs.readFileSync(path.join(__dirname,'../generated/selected.json')));
  const options={controller:'pd',feedback:'eskf',scenario:'schedule',seconds:.2,preview:false,
    commands:[{t:0,speed:0,altitude:20},{t:.04,speed:3,altitude:21}]};
  const fast=await R.run(ca,data,options),original=await R.run(ca,{...data,numeric:null},options);
  close([fast.final],[original.final],'closed-loop final');
  assert.equal(fast.trace.length,original.trace.length);
  for(let i=0;i<fast.trace.length;i++)close([fast.trace[i].state,fast.trace[i].estimate],
    [original.trace[i].state,original.trace[i].estimate],'closed-loop trace');
  assert.ok(Math.abs(fast.metrics.electrical_energy_J-original.metrics.electrical_energy_J)<1e-7);
});
