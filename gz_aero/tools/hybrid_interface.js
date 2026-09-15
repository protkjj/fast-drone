/* Interface-separated INDI, for the body +x thrust plant.
 * NMPC owns [total thrust, angular acceleration]; only this module owns motors.
 * We solve the increment in rotor-force coordinates: Δf = (∂f/∂n) Δn.
 * Unlike clipping G^-1 Δν afterwards, bounded allocation preserves total thrust.
 * Aerodynamic, rigid-body and rotor-gyro moments enter via measured acceleration,
 * not an accumulated requested moment. This is a local incremental approximation.
 */
const HYBRID_INDI = {filterHz:50};
let hybridState=null, hybridStat=null;
function resetHybridInterface(){ hybridState=null; hybridStat=null; }
function hybridSnapshot(){
  return {state:hybridState?JSON.parse(JSON.stringify(hybridState)):null,
          stat:hybridStat?JSON.parse(JSON.stringify(hybridStat)):null};
}
function hybridRestore(saved){
  const copy=saved?JSON.parse(JSON.stringify(saved)):{};
  hybridState=copy.state||null; hybridStat=copy.stat||null;
}
function hybridRotorForce(n, vax){
  return P.k_T*n*n*Math.max(0,1-vax/((n/(2*Math.PI))*P.D_prop+EPS)/P.J_max);
}
function hybridForceMatrix(){
  if(P.thrust_axis!=='x') throw Error('Hybrid INDI requires body +x thrust geometry');
  return [P.rotor_directions.map(d=>d*P.k_Q/P.k_T/P.Ixx),
          P.rotor_positions.map(r=>r[2]/P.Iyy),
          P.rotor_positions.map(r=>-r[1]/P.Izz)];
}
// Small dense system with pivoting. KKT matrices are indefinite, so not Cholesky.
function hybridLinearSolve(A,b){
  const n=b.length, rows=A.map((row,i)=>[...row,b[i]]);
  for(let k=0;k<n;k++){
    let p=k;for(let i=k+1;i<n;i++)if(Math.abs(rows[i][k])>Math.abs(rows[p][k]))p=i;
    if(Math.abs(rows[p][k])<1e-12)return null;
    [rows[k],rows[p]]=[rows[p],rows[k]];
    for(let i=k+1;i<n;i++){
      const scale=rows[i][k]/rows[k][k];
      for(let j=k;j<=n;j++)rows[i][j]-=scale*rows[k][j];
    }
  }
  const x=Array(n).fill(0);
  for(let i=n-1;i>=0;i--){
    let sum=rows[i][n];for(let j=i+1;j<n;j++)sum-=rows[i][j]*x[j];
    x[i]=sum/rows[i][i];
  }
  return x;
}
// min ||B f - angularTarget||², subject to sum(f)=T and 0<=f_i<=cap_i.
// Four rotors => only 3^4 active faces. The feasible unconstrained solution exits
// immediately; otherwise enumerate faces, giving a global minimum of this QP.
function hybridAllocate(total,angularTarget,B,caps){
  const maxTotal=caps.reduce((a,b)=>a+b,0), Tc=Math.max(0,Math.min(maxTotal,total));
  if(Tc<=1e-12)return [0,0,0,0];
  if(maxTotal-Tc<=1e-12)return caps.slice();
  const H=Array.from({length:4},(_,i)=>Array.from({length:4},(_,j)=>
    B.reduce((sum,row)=>sum+row[i]*row[j],0)));
  const g=Array.from({length:4},(_,i)=>B.reduce((sum,row,k)=>sum+row[i]*angularTarget[k],0));
  let best=null,bestCost=Infinity;
  for(let face=0;face<81;face++){
    let code=face,remain=Tc;const free=[],f=[0,0,0,0];
    for(let i=0;i<4;i++){
      const status=code%3;code=Math.floor(code/3);
      if(status===0)free.push(i);else if(status===2){f[i]=caps[i];remain-=f[i];}
    }
    if(remain < -1e-9 || remain>free.reduce((sum,i)=>sum+caps[i],0)+1e-9)continue;
    const m=free.length;
    if(m){
      const A=free.map(i=>[...free.map(j=>H[i][j]),1]);
      A.push([...free.map(()=>1),0]);
      const rhs=free.map(i=>g[i]-f.reduce((sum,v,j)=>sum+H[i][j]*v,0));rhs.push(remain);
      const solved=hybridLinearSolve(A,rhs);if(!solved)continue;
      free.forEach((i,k)=>{f[i]=solved[k];});
    }else if(Math.abs(remain)>1e-9)continue;
    if(f.some((v,i)=>v < -1e-8 || v>caps[i]+1e-8))continue;
    const cost=B.reduce((sum,row,k)=>sum+(row.reduce((a,b,i)=>a+b*f[i],0)-angularTarget[k])**2,0);
    if(cost<bestCost){bestCost=cost;best=f;}
    if(face===0)break;
  }
  if(!best)throw Error('Hybrid allocation: no feasible total-thrust solution');
  return best.map((f,i)=>Math.max(0,Math.min(caps[i],f)));
}
function hybridIndi(x,requested,dt){
  const omega=x.slice(10,13),vax=axialInflow(x),B=hybridForceMatrix();
  const forces=x.slice(13,17).map(n=>hybridRotorForce(n,vax));
  if(!hybridState){
    // No previous IMU sample exists at reset: start with zero angular acceleration.
    // Never read xdot's true acceleration as a substitute for measurement.
    hybridState={omega:omega.slice(),forces:forces.slice(),filtered:forces.slice(),alpha:[0,0,0]};
  }else{
    const a=1-Math.exp(-2*Math.PI*HYBRID_INDI.filterHz*dt);
    for(let k=0;k<3;k++)hybridState.alpha[k]+=a*((omega[k]-hybridState.omega[k])/dt-hybridState.alpha[k]);
    // Δω/dt covers the previous interval; use its midpoint actuator force and
    // the same LPF to avoid treating motor/filter delay as an external moment.
    for(let i=0;i<4;i++)hybridState.filtered[i]+=a*((forces[i]+hybridState.forces[i])/2-hybridState.filtered[i]);
  }
  const s=hybridState, measuredT=forces.reduce((a,b)=>a+b,0);
  const delta=[requested[0]-measuredT,...requested.slice(1).map((v,k)=>v-s.alpha[k])];
  const rotorAlpha=B.map(row=>row.reduce((sum,b,i)=>sum+b*s.filtered[i],0));
  const target=rotorAlpha.map((v,k)=>v+delta[k+1]);
  const caps=forces.map(()=>hybridRotorForce(P.n_max,vax));
  const allocated=hybridAllocate(measuredT+delta[0],target,B,caps);
  const motors=allocated.map(f=>nFromThrust(f,vax));
  const applied=[allocated.reduce((a,b)=>a+b,0),...B.map((row,k)=>
    s.alpha[k]+row.reduce((sum,b,i)=>sum+b*(allocated[i]-s.filtered[i]),0))];
  hybridStat={t:T,requested:requested.slice(),applied,measured:[measuredT,...s.alpha],delta,
    residual:requested.map((v,k)=>v-applied[k]),forces:allocated,
    limited:Math.hypot(...requested.map((v,k)=>v-applied[k]))>1e-5};
  s.omega=omega.slice();s.forces=forces.slice();
  return motors;
}
