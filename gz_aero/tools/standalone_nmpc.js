/* 단독 NMPC: 17상태 → 모터 회전수 4개를 직접 최적화한다.
 * INDI/PID/LQR 제어 출력은 사용하지 않는다. LQR 표에서는 평형 자세·회전수만 읽는다.
 * 동일한 xdot/RK4로 비선형 예측 → 유한차분 선형화 → 압축 SQP → 실제 비용 선탐색.
 * 매 주기 SQP 1회인 RTI 근사이며, Python CasADi/IPOPT와 같은 해라는 뜻은 아니다.
 */
const MOTOR_MPC = {N:12, dt:0.05, rate:0.05, maxStep:0.01,
  zCap:5, vzMax:6, speedLead:3, terminal:8, trust:0.08,
  R:0.000002, Rdu:0.00002, regularization:0.001,
  // 고도, 속도, 쿼터니언, 각속도, 실제 모터 회전수 순서. XY 위치는 추종하지 않는다.
  indices:[2,3,4,5,6,7,8,9,10,11,12,13,14,15,16],
  weights:[20,10,10,15,400,400,400,400,10,20,20,0.00002,0.00002,0.00002,0.00002]};
let motorU=null, motorOut=null, motorLast=-1e9, motorStat=null, motorFailures=0;

function resetMotorMpc(){
  motorU=null; motorOut=null; motorLast=-1e9; motorStat=null; motorFailures=0;
}
function motorSnapshot(){
  return {u:motorU?Array.from(motorU):null,out:motorOut?motorOut.slice():null,
    age:T-motorLast,stat:motorStat?{...motorStat}:null,failures:motorFailures};
}
function motorRestore(s){
  resetMotorMpc(); if(!s)return;
  motorU=s.u?Float64Array.from(s.u):null; motorOut=s.out?s.out.slice():null;
  motorLast=T-s.age; motorStat=s.stat?{...s.stat}:null; motorFailures=s.failures;
}
function motorPredict(x,u,wind,dt){
  // 20 ms 모터 시정수를 큰 RK4 보폭으로 건너뛰지 않는다.
  return rk4(x,u,wind,P,dt,Math.ceil(dt/Math.min(MOTOR_MPC.maxStep,P.tau_m/2)));
}
function motorReference(x,cmd){
  const C=MOTOR_MPC, speed=Math.min(Math.hypot(x[3],x[4])+C.speedLead,cmd.spd,LQ.V_max_table);
  const [target,u]=lqrPick(Math.max(0,speed));
  const dz=cmd.alt-x[2];
  target[2]=x[2]+Math.max(-C.zCap,Math.min(C.zCap,dz));
  target[5]=Math.max(-C.vzMax,Math.min(C.vzMax,dz));
  return {target,u,wind:windNow()};
}
function motorRollout(x0,U,ref,previous){
  const C=MOTOR_MPC, scale=P.n_max, states=[x0]; let cost=0;
  for(let k=0;k<C.N;k++){
    const u=Array.from(U.slice(4*k,4*k+4),v=>v*scale);
    const x=motorPredict(states[k],u,ref.wind,C.dt);
    if(!x.every(Number.isFinite))return {cost:Infinity,states};
    states.push(x);
    const sign=x.slice(6,10).reduce((s,v,i)=>s+v*ref.target[6+i],0)<0?-1:1;
    const terminal=k===C.N-1?C.terminal:1;
    C.indices.forEach((r,j)=>{
      const desired=ref.target[r]*(r>=6 && r<=9?sign:1);
      cost+=terminal*C.weights[j]*(x[r]-desired)**2;
    });
    for(let i=0;i<4;i++){
      const prev=k?U[4*(k-1)+i]*scale:previous[i];
      cost+=C.R*(u[i]-ref.u[i])**2+C.Rdu*(u[i]-prev)**2;
    }
  }
  return {cost,states};
}
function motorQuadratic(states,U,ref,previous){
  const C=MOTOR_MPC, n=4*C.N, scale=P.n_max;
  const H=new Float64Array(n*n), g=new Float64Array(n);
  let sensitivity=new Float64Array(NX*n);
  for(let k=0;k<C.N;k++){
    const x=states[k], next=states[k+1], u=Array.from(U.slice(4*k,4*k+4),v=>v*scale);
    const A=new Float64Array(NX*NX), B=new Float64Array(NX*4);
    for(let j=0;j<NX;j++){
      const h=1e-5*(1+Math.abs(x[j])), perturbed=x.slice();perturbed[j]+=h;
      const y=motorPredict(perturbed,u,ref.wind,C.dt);
      for(let r=0;r<NX;r++)A[r*NX+j]=(y[r]-next[r])/h;
    }
    for(let j=0;j<4;j++){
      const h=1e-5, perturbed=u.slice();perturbed[j]+=h*scale;
      const y=motorPredict(x,perturbed,ref.wind,C.dt);
      for(let r=0;r<NX;r++)B[r*4+j]=(y[r]-next[r])/h;
    }
    const propagated=new Float64Array(NX*n), active=4*(k+1);
    for(let r=0;r<NX;r++){
      for(let j=0;j<4*k;j++){
        let sum=0;for(let q=0;q<NX;q++)sum+=A[r*NX+q]*sensitivity[q*n+j];
        propagated[r*n+j]=sum;
      }
      for(let j=0;j<4;j++)propagated[r*n+4*k+j]=B[r*4+j];
    }
    sensitivity=propagated;
    const sign=next.slice(6,10).reduce((s,v,i)=>s+v*ref.target[6+i],0)<0?-1:1;
    const terminal=k===C.N-1?C.terminal:1;
    C.indices.forEach((r,index)=>{
      const weight=C.weights[index]*terminal;
      const error=next[r]-ref.target[r]*(r>=6 && r<=9?sign:1), off=r*n;
      for(let a=0;a<active;a++){
        const wa=weight*sensitivity[off+a];g[a]+=wa*error;
        for(let b=0;b<=a;b++)H[a*n+b]+=wa*sensitivity[off+b];
      }
    });
  }
  for(let a=0;a<n;a++)for(let b=0;b<a;b++)H[b*n+a]=H[a*n+b];
  for(let k=0;k<C.N;k++)for(let i=0;i<4;i++){
    const j=4*k+i, prev=k?U[j-4]:previous[i]/scale;
    const R=C.R*scale*scale, Rd=C.Rdu*scale*scale;
    g[j]+=R*(U[j]-ref.u[i]/scale)+Rd*(U[j]-prev);H[j*n+j]+=R+Rd;
    if(k){g[j-4]-=Rd*(U[j]-prev);H[(j-4)*n+j-4]+=Rd;
      H[j*n+j-4]-=Rd;H[(j-4)*n+j]-=Rd;}
    H[j*n+j]+=C.regularization;
  }
  return {H,g};
}
function motorBoxStep(H,g,U){
  const n=U.length, C=MOTOR_MPC, step=new Float64Array(n);
  const low=Array.from(U,v=>Math.max(P.n_min/P.n_max-v,-C.trust));
  const high=Array.from(U,v=>Math.min(1-v,C.trust));
  const objective=d=>{
    let value=0;
    for(let i=0;i<n;i++){
      value+=g[i]*d[i]+0.5*H[i*n+i]*d[i]*d[i];
      for(let j=0;j<i;j++)value+=H[i*n+j]*d[i]*d[j];
    }return value;
  };
  for(let iteration=0;iteration<4;iteration++){
    const grad=Float64Array.from(g), free=[];
    for(let i=0;i<n;i++){
      for(let j=0;j<n;j++)grad[i]+=H[i*n+j]*step[j];
      if((step[i]<=low[i]+1e-10 && grad[i]>0)||(step[i]>=high[i]-1e-10 && grad[i]<0))continue;
      free.push(i);
    }
    if(!free.length)break;
    const m=free.length, subH=new Float64Array(m*m), rhs=new Float64Array(m);
    for(let i=0;i<m;i++){
      rhs[i]=-grad[free[i]];
      for(let j=0;j<m;j++)subH[i*m+j]=H[free[i]*n+free[j]];
    }
    const direction=cholSolve(subH,rhs,m);if(!direction)break;
    const before=objective(step);let accepted=false;
    for(let alpha=1;alpha>=1/128;alpha/=2){
      const candidate=Float64Array.from(step);
      free.forEach((j,i)=>candidate[j]=Math.max(low[j],Math.min(high[j],step[j]+alpha*direction[i])));
      if(objective(candidate)<before-1e-10){step.set(candidate);accepted=true;break;}
    }
    if(!accepted)break;
  }
  return step;
}
function motorSolve(x,cmd){
  const start=performance.now(), C=MOTOR_MPC, n=4*C.N, scale=P.n_max;
  const ref=motorReference(x,cmd), previous=motorOut||x.slice(13,17);
  const flat=Float64Array.from({length:n},(_,j)=>ref.u[j%4]/scale);
  let U=motorU?Float64Array.from(motorU):flat;
  if(motorU)for(let k=0;k<C.N-1;k++)for(let i=0;i<4;i++)U[4*k+i]=motorU[4*(k+1)+i];
  const clip=v=>Math.max(P.n_min/scale,Math.min(1,v));U=U.map(clip);
  let rollout=motorRollout(x,U,ref,previous);
  const flatRollout=motorRollout(x,flat,ref,previous);
  if(flatRollout.cost<rollout.cost){U=flat;rollout=flatRollout;}
  if(!Number.isFinite(rollout.cost))return null;
  const initialCost=rollout.cost, {H,g}=motorQuadratic(rollout.states,U,ref,previous);
  if(!H.every(Number.isFinite)||!g.every(Number.isFinite))return null;
  const step=motorBoxStep(H,g,U);let accepted=false;
  // QP가 좋아졌더라도 실제 비선형 예측의 비용이 나빠지면 적용하지 않는다.
  for(let alpha=1;alpha>=1/128;alpha/=2){
    const candidate=U.map((v,i)=>clip(v+alpha*step[i]));
    const next=motorRollout(x,candidate,ref,previous);
    if(next.cost<rollout.cost-1e-9){U=candidate;rollout=next;accepted=true;break;}
  }
  motorU=U;
  motorStat={ms:performance.now()-start,initialCost,cost:rollout.cost,accepted,failed:false};
  return Array.from(U.slice(0,4),v=>v*scale);
}
function controlMotorNmpc(x,cmd){
  if(motorOut && T-motorLast<MOTOR_MPC.rate-1e-9)return motorOut.slice();
  const result=motorSolve(x,cmd);motorLast=T;
  if(result && result.every(Number.isFinite)){
    motorOut=result;motorFailures=0;
  }else{
    motorFailures++;motorStat={failed:true,failures:motorFailures};
    // 숨은 LQR 전환은 없다. 한두 번의 계산 실패는 직전 모터 명령을 유지한다.
    if(!motorOut)motorOut=x.slice(13,17).map(v=>Math.max(P.n_min,Math.min(P.n_max,v)));
    if(motorFailures>=3){physicsFailure='단독 NMPC 풀이 연속 실패';running=false;health.diverged=true;}
  }
  return motorOut.slice();
}
