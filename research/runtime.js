/* Shared Node / Web Worker experiment runner. All dynamics come from model.py. */
(function (root) {
  'use strict';
  const DT = .001, PERIOD = 20;
  const clamp = (x, a, b) => Math.min(b, Math.max(a, x));
  const norm = x => Math.hypot(...x);
  const add = (a, b) => a.map((v, i) => v+b[i]);
  const sub = (a, b) => a.map((v, i) => v-b[i]);
  const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  function rotate(q, v, inverse=false) {
    const a = q.slice(0,3).map(x => inverse ? -x : x);
    const t = cross(a,v).map(x => 2*x);
    return add(v, add(t.map(x => q[3]*x),cross(a,t)));
  }
  function random(seed) {
    let state=seed>>>0;
    const uniform = () => { state=(Math.imul(1664525,state)+1013904223)>>>0; return (state+.5)/4294967296; };
    return () => Math.sqrt(-2*Math.log(uniform()))*Math.cos(2*Math.PI*uniform());
  }
  function linearSolve(matrix, rhs) {
    const n=rhs.length, a=matrix.map((row,i)=>[...row,rhs[i]]);
    for(let j=0;j<n;j++) {
      let pivot=j;
      for(let i=j+1;i<n;i++) if(Math.abs(a[i][j])>Math.abs(a[pivot][j])) pivot=i;
      [a[j],a[pivot]]=[a[pivot],a[j]];
      if(Math.abs(a[j][j])<1e-18) return null;
      const d=a[j][j]; for(let k=j;k<=n;k++) a[j][k]/=d;
      for(let i=0;i<n;i++) if(i!==j) {
        const f=a[i][j]; for(let k=j;k<=n;k++) a[i][k]-=f*a[j][k];
      }
    }
    return a.map(row=>row[n]);
  }
  // Enumerate the 3^4 free/lower/upper faces of the four-actuator least squares.
  // This preserves coupled thrust/torque priorities when an actuator saturates.
  function boundedIncrement(g, error, lower, upper, weights) {
    const a=g.map((row,i)=>row.map(x=>x*weights[i]));
    const b=error.map((x,i)=>x*weights[i]);
    const ridge=1e-12;
    let best=null, bestCost=Infinity;
    for(let face=0;face<81;face++) {
      let bits=face, u=[0,0,0,0], free=[];
      for(let i=0;i<4;i++) { const mode=bits%3; bits=Math.floor(bits/3);
        if(mode===0) free.push(i); else u[i]=mode===1?lower[i]:upper[i]; }
      const remainder=b.map((v,r)=>v-a[r].reduce((s,x,c)=>s+x*u[c],0));
      if(free.length) {
        const h=free.map(i=>free.map(j=>a.reduce((s,row)=>s+row[i]*row[j],0)+(i===j?ridge:0)));
        const rhs=free.map(i=>a.reduce((s,row,r)=>s+row[i]*remainder[r],0));
        const solution=linearSolve(h,rhs); if(!solution) continue;
        free.forEach((i,k)=>u[i]=solution[k]);
      }
      if(u.some((v,i)=>v<lower[i]-1e-7||v>upper[i]+1e-7)) continue;
      const cost=a.reduce((s,row,r)=>s+(row.reduce((v,x,c)=>v+x*u[c],0)-b[r])**2,0)+ridge*u.reduce((s,x)=>s+x*x,0);
      if(cost<bestCost) {bestCost=cost;best=u;}
    }
    if(!best) throw new Error('Bounded INDI allocation has no finite solution');
    return best;
  }

  // Four-actuator QP: exact total thrust, bounded rotor forces, least-squares
  // angular target. The tiny ridge resolves singular zero-speed roll authority.
  function allocateThrust(total,target,matrix,caps,anchor) {
    const sum=a=>a.reduce((s,v)=>s+v,0),thrust=clamp(total,0,sum(caps)),ridge=1e-10;
    if(thrust<1e-12)return [0,0,0,0];
    if(sum(caps)-thrust<1e-12)return caps.slice();
    const h=Array.from({length:4},(_,i)=>Array.from({length:4},(_,j)=>
      matrix.reduce((s,row)=>s+row[i]*row[j],0)+(i===j?ridge:0)));
    const gradient=Array.from({length:4},(_,i)=>matrix.reduce((s,row,k)=>s+row[i]*target[k],0)+ridge*anchor[i]);
    let best=null,cost=Infinity;
    for(let face=0;face<81;face++){
      let bits=face,left=thrust;const force=[0,0,0,0],free=[];
      for(let i=0;i<4;i++){
        const mode=bits%3;bits=Math.floor(bits/3);
        if(mode===0)free.push(i);else if(mode===2){force[i]=caps[i];left-=caps[i];}
      }
      if(left < -1e-8 || left>sum(free.map(i=>caps[i]))+1e-8)continue;
      if(free.length){
        const kkt=free.map(i=>[...free.map(j=>h[i][j]),1]);kkt.push([...free.map(()=>1),0]);
        const rhs=free.map(i=>gradient[i]-force.reduce((s,f,j)=>s+h[i][j]*f,0));rhs.push(left);
        const solved=linearSolve(kkt,rhs);if(!solved)continue;
        free.forEach((i,k)=>force[i]=solved[k]);
      }else if(Math.abs(left)>1e-8)continue;
      if(force.some((f,i)=>f < -1e-7 || f>caps[i]+1e-7))continue;
      const value=matrix.reduce((s,row,k)=>s+(sum(row.map((v,i)=>v*force[i]))-target[k])**2,0)
        +ridge*force.reduce((s,f,i)=>s+(f-anchor[i])**2,0);
      if(value<cost){cost=value;best=force;}
      if(face===0)break;
    }
    if(!best)throw new Error('No feasible total-thrust allocation');
    return best.map((f,i)=>clamp(f,0,caps[i]));
  }

  function motorDiagnostics(d){
    return {thrust_N:d[0],torque_Nm:d[1],current_A:d[2],rotor_accel_rad_s2:d[3],
      voltage_V:d[4][0],bus_current_A:d[5][0],power_W:d[6][0],
      requested_current_A:d[17],current_limit_A:d[18][0],voltage_current_cap_A:d[19],
      current_limited:d[20].map(Boolean),voltage_limited:d[21].map(Boolean),
      coasting:d[22].map(Boolean),tracking_limited:d[23].map(Boolean)};
  }

  function makeBindings(ca, data) {
    const decode=group=>Object.fromEntries(Object.entries(group).map(([k,v])=>[k,ca.Function.deserialize(v)]));
    const functions=decode(data.functions), estimator=decode(data.estimator);
    const solvers=Object.fromEntries(Object.entries(data.solvers).map(([k,v])=>[k,ca.Function.deserialize(v.function)]));
    // elements() includes structural zeros, unlike nonzeros(). Matrix values
    // returned through the 3.8.0 binding are indexed Proxies. Calling delete()
    // on them does not unregister the original finalizer token and can cause
    // a double free at GC. Let the package's FinalizationRegistry own matrices.
    const array=dm=>dm.elements();
    const call=(fn,...args)=>fn.call(args.map(v=>ca.DM(v))).map(array);
    const dispose=()=>[solvers,functions,estimator].forEach(group=>Object.values(group).forEach(fn=>fn.delete()));
    return {functions,estimator,solvers,call,array,dispose};
  }

  // Warm-start grid nodes are 50 ms apart, but the controller advances 20 ms.
  // Shifting a whole node predicts 30 ms too far ahead on every control update.
  function shiftPrediction(values, width, fraction) {
    const blocks=values.length/width;
    return values.map((v,i)=>{
      const block=Math.floor(i/width),next=Math.min(block+1,blocks-1)*width+i%width;
      return (1-fraction)*v+fraction*values[next];
    });
  }

  class Optimizer {
    constructor(ca, binding, data, kind) {
      this.ca=ca; this.binding=binding; this.meta=data.solvers[kind]; this.solver=binding.solvers[kind];
      this.previous=this.meta.hover.slice(); this.command=this.previous.slice();this.guess=null; this.stats=[];
      this.stateScale=this.meta.state_scaling||Array(this.meta.nx).fill(1);
      this.commandScale=this.meta.command_scaling||[1,1,1,1];
      this.constraintScale=this.meta.constraint_scaling||Array.from({length:this.meta.lbg.length},(_,i)=>this.stateScale[i%this.meta.nx]);
      this.lamX=null; this.lamG=null;
    }
    solve(state, refs, voltage, wind) {
      const m=this.meta, x=state.slice(0,m.nx);
      if(!this.guess) {
        const scaledX=x.map((v,i)=>v/this.stateScale[i]);
        const scaledU=m.hover.map((v,i)=>v/this.commandScale[i]);
        this.guess=[...Array.from({length:m.N+1},()=>scaledX).flat(),...Array.from({length:m.N},()=>scaledU).flat()];
      }
      const parameters=[...x,...refs.flat(),...this.previous,...wind,voltage];
      const begin=performance.now();
      let result, status, values, residual=Infinity, success=false;
      try {
        const input={x0:this.guess,p:parameters,lbx:m.lbx,ubx:m.ubx,lbg:m.lbg,ubg:m.ubg};
        if(this.lamX) {input.lam_x0=this.lamX; input.lam_g0=this.lamG;}
        const packed=Object.fromEntries(Object.entries(input).map(([k,v])=>[k,this.ca.DM(v)]));
        result=this.solver.call(packed);
        values=this.binding.array(result.x);
        // Keep the pre-normalization acceptance gate in physical units. Scaling
        // must not disguise a large motor-state defect as a small dimensionless one.
        residual=Math.max(0,...this.binding.array(result.g).map((v,i)=>
          Math.max(m.lbg[i]-v,v-m.ubg[i])*this.constraintScale[i]));
        status=this.solver.stats();
        success=status.success && residual<1e-3 && values.every(Number.isFinite);
      } catch(error) {status={return_status:String(error.message||error),iter_count:0};}
      const elapsed=performance.now()-begin;
      this.stats.push({ms:elapsed,success,status:status.return_status,iterations:status.iter_count,residual});
      if(success) {
        const shift=m.control_dt_s/m.prediction_dt_s;
        const lamX=this.binding.array(result.lam_x),lamG=this.binding.array(result.lam_g);
        this.lamX=[...shiftPrediction(lamX.slice(0,m.state_count),m.nx,shift),
          ...shiftPrediction(lamX.slice(m.state_count),4,shift)];
        this.lamG=[...shiftPrediction(lamG.slice(0,m.state_count),m.nx,shift),
          ...shiftPrediction(lamG.slice(m.state_count),160,shift)];
        const controls=values.slice(m.state_count);
        this.previous=controls.slice(0,4).map((v,i)=>clamp(v*this.commandScale[i],m.lower[i],m.upper[i]));
        this.command=this.previous.slice();
        // Shift primal warm start. Previous-input cost uses the ACTUAL last request,
        // never a reset-to-hover placeholder on every solve.
        const states=values.slice(0,m.state_count);
        const shifted=shiftPrediction(states,m.nx,shift);
        for(let k=0;k<=m.N;k++){
          const offset=k*m.nx+6,q=shifted.slice(offset,offset+4),length=norm(q);
          if(length>1e-8)q.forEach((v,i)=>shifted[offset+i]=v/length);
        }
        this.guess=[...shifted,...shiftPrediction(controls,4,shift)];
      }
      // Declared fallback: hold the last bounded command; no hidden PD/INDI for NMPC.
      return this.command.slice();
    }
  }

  class Estimator {
    constructor(binding, data, initial) {
      this.b=binding; this.config=data.sensors;
      this.state=[...initial.slice(0,10),0,0,0,0,0,0];
      // Surveyed launch pose: 0.1 m position / 0.05 m/s velocity / 0.01 rad
      // attitude standard deviations. Same initialization for BOTH controllers.
      const diagonal=[.01,.01,.01,.0025,.0025,.0025,.0001,.0001,.0001,.0025,.0025,.0025,.000025,.000025,.000025];
      this.cov=Array.from({length:225},(_,i)=>i%16===0?diagonal[i/16]:0);
      this.nodes=new Map([[0,[this.state.slice(),this.cov.slice()]]]);
      this.inputs=new Map(); this.tick=0; this.replayed=0; this.updates=0;
    }
    predict(imu) {
      this.inputs.set(this.tick,imu.slice());
      [this.state,this.cov]=this.b.call(this.b.estimator.predict,this.state,this.cov,imu);
      this.tick++;
      this.nodes.set(this.tick,[this.state.slice(),this.cov.slice()]);
      const oldest=this.tick-200;
      this.nodes.delete(oldest); this.inputs.delete(oldest);
    }
    gps(captureTick, measurement) {
      const old=this.nodes.get(captureTick);
      if(!old) throw new Error('GPS timestamp is outside the ESKF history');
      let [state,cov,nis]=this.b.call(this.b.estimator.gps,old[0],old[1],measurement);
      this.lastGPS={capture_t_s:captureTick*DT,arrival_t_s:this.tick*DT,
        measurement:measurement.slice(),innovation:sub(measurement,old[0].slice(0,6)),nis:nis[0]};
      this.nodes.set(captureTick,[state.slice(),cov.slice()]);
      for(let i=captureTick;i<this.tick;i++) {
        [state,cov]=this.b.call(this.b.estimator.predict,state,cov,this.inputs.get(i));
        this.nodes.set(i+1,[state.slice(),cov.slice()]); this.replayed++;
      }
      this.state=state; this.cov=cov; this.updates++;
    }
    feedback(gyro,rpm) { return [...this.state.slice(0,10),...sub(gyro,this.state.slice(13,16)),...rpm]; }
  }

  class INDI {
    constructor(binding,data) {
      this.b=binding; this.p=data.profile;
      this.n=data.initial.slice(13,17); this.nRaw=this.n.slice(); this.gyro=[0,0,0];
      this.alpha=[0,0,0]; this.thrust=this.p.mass_kg*this.p.g;
      this.thrustRaw=this.thrust;
      this.max=data.solvers.hybrid.max_rotor_rad_s;
      this.lpf=1-Math.exp(-2*Math.PI*50*DT); this.samples=0; this.saturated=0;
      const [force,torque]=binding.call(binding.functions.rotors,this.n,[0]);
      this.forceRaw=force.slice();this.torqueRaw=torque.slice();
      this.force=force.slice();this.torque=torque.slice();this.last=null;
    }
    update(state, desired, rateMeasurement=state.slice(10,13)) {
      // Differentiate the measured gyro, not the bias-corrected state: a GPS
      // update to estimated gyro bias must not create a fictitious alpha impulse.
      const rpm=state.slice(13,17), gyro=rateMeasurement;
      const axial=rotate(state.slice(6,10),state.slice(3,6),true)[0];
      const [force,torque]=this.b.call(this.b.functions.rotors,rpm,[axial]);
      const midpoint=rpm.map((v,i)=>(v+this.nRaw[i])/2);
      this.n=this.n.map((v,i)=>v+this.lpf*(midpoint[i]-v));
      this.alpha=this.alpha.map((v,i)=>v+this.lpf*((gyro[i]-this.gyro[i])/DT-v));
      this.force=this.force.map((v,i)=>v+this.lpf*((force[i]+this.forceRaw[i])/2-v));
      this.torque=this.torque.map((v,i)=>v+this.lpf*((torque[i]+this.torqueRaw[i])/2-v));
      this.forceRaw=force.slice();this.torqueRaw=torque.slice();
      this.thrust=this.force.reduce((s,v)=>s+v,0);
      this.gyro=gyro.slice(); this.nRaw=rpm.slice();
      const inertia=this.p.inertia_kg_m2,a=this.p.arm_m/Math.sqrt(2),dirs=[1,-1,1,-1];
      const ys=[a,-a,-a,a],zs=[a,a,-a,-a],dot=(x,y)=>x.reduce((s,v,i)=>s+v*y[i],0);
      const angular=(f,q)=>[dot(dirs,q)/inertia[0],dot(zs,f)/inertia[1],-dot(ys,f)/inertia[2]];
      const baseline=angular(this.force,this.torque),target=add(baseline,sub(desired.slice(1),this.alpha));
      const [caps]=this.b.call(this.b.functions.rotors,[this.max,this.max,this.max,this.max],[axial]);
      const capacity=caps.reduce((s,v)=>s+v,0),total=clamp(desired[0],0,capacity);
      const evaluate=f=>{
        const [n]=this.b.call(this.b.functions.inverse_thrust,f,[axial],[this.max]);
        const [actual,q,df,dq]=this.b.call(this.b.functions.rotors,n,[axial]);
        const alpha=angular(actual,q);
        const matrix=[dq.map((v,i)=>dirs[i]*(df[i]>1e-10?v/df[i]:0)/inertia[0]),
                      zs.map(v=>v/inertia[1]),ys.map(v=>-v/inertia[2])];
        return {n,force:actual,alpha,matrix,cost:sub(alpha,target).reduce((s,v)=>s+v*v,0)};
      };
      // Start on the equality, even with previously stopped rotors. Nonlinear
      // map validation prevents a large Δn approximation from inventing thrust.
      let candidate=evaluate(caps.map(v=>capacity>0?total*v/capacity:0));
      for(let iteration=0;iteration<4&&total>1e-12;iteration++){
        const linearTarget=target.map((v,k)=>v-candidate.alpha[k]+dot(candidate.matrix[k],candidate.force));
        const next=allocateThrust(total,linearTarget,candidate.matrix,caps,this.force);
        let improved=false;
        for(const fraction of [1,.5,.25,.125]){
          const trial=evaluate(candidate.force.map((v,i)=>v+fraction*(next[i]-v)));
          if(trial.cost<candidate.cost-1e-10){candidate=trial;improved=true;break;}
        }
        if(!improved)break;
      }
      const cmd=candidate.n;
      const allocated=[candidate.force.reduce((s,v)=>s+v,0),...add(this.alpha,sub(candidate.alpha,baseline))];
      const residual=sub(desired,allocated);
      this.last={requested:desired.slice(),allocated,measured:[this.thrust,...this.alpha],residual,
        rotor_force_command_N:candidate.force.slice(),thrust_cap_N:capacity,
        limited:Math.abs(residual[0])>1e-4||norm(residual.slice(1))>1e-3,
        note:'Nominal command-side allocation, not instantaneous thrust; physical motor limits and lag remain.'};
      this.samples++; if(cmd.some(v=>v<1e-6||v>this.max-1e-6)) this.saturated++;
      return cmd;
    }
  }

  function reference(t, config) {
    // This changes only the REFERENCE. The actual velocity is always integrated.
    const vx=config.scenario==='hover'?0:(t<1?0:config.speed);
    return [vx,0,0,config.altitude??20];
  }
  function referenceHorizon(t,config) {
    return Array.from({length:21},(_,k)=>reference(config.preview===false?t:t+k*.05,config));
  }
  function validateOptions(options={}) {
    const cfg={controller:'hybrid',feedback:'truth',scenario:'step',seconds:4,speed:3,altitude:20,
      seed:42,preview:true,log_hz:50,scales:[1,1,1,1,1],...options};
    if(!['hybrid','nmpc'].includes(cfg.controller))throw new Error('Unknown controller');
    if(!['truth','eskf'].includes(cfg.feedback))throw new Error('Unknown feedback');
    if(!['hover','step','gust'].includes(cfg.scenario))throw new Error('Unknown scenario');
    if(!Number.isFinite(cfg.seconds)||cfg.seconds<.001||cfg.seconds>120)throw new Error('Duration must be 0.001–120 seconds');
    if(cfg.scenario==='step'&&cfg.seconds<1.1)throw new Error('속도 계단 시험은 1초 입력 이후를 포함하도록 1.1초 이상 필요합니다.');
    if(cfg.scenario==='gust'&&cfg.seconds<3.1)throw new Error('외란 시험은 2–3초 외란과 종료 이후를 포함하도록 3.1초 이상 필요합니다.');
    if(!Number.isFinite(cfg.speed)||cfg.speed<0||cfg.speed>100)throw new Error('Reference speed must be 0–100 m/s');
    if(!Number.isFinite(cfg.altitude)||cfg.altitude<1||cfg.altitude>1000)throw new Error('고도는 1–1000 m 범위입니다.');
    if(!Array.isArray(cfg.scales)||cfg.scales.length!==5||cfg.scales.some(v=>!Number.isFinite(v)||v<.5||v>1.5))throw new Error('Invalid model-error scales');
    if(!Number.isInteger(cfg.seed)||cfg.seed<0||cfg.seed>4294967295)throw new Error('Seed must be an unsigned 32-bit integer');
    if(typeof cfg.preview!=='boolean')throw new Error('Preview must be true or false');
    if(![50,1000].includes(cfg.log_hz)||cfg.log_hz===1000&&cfg.seconds>20)throw new Error('로그는 50 Hz 또는 1 kHz입니다. 상세 1 kHz 로그는 20초 이하로 제한합니다.');
    return cfg;
  }
  function environment(t, config) {
    if(config.scenario==='gust' && t>=2 && t<3) return [0,3,0,.015,.025,0];
    return [0,0,0,0,0,0];
  }
  function percentile(values, q) {
    if(!values.length) return null;
    const sorted=values.slice().sort((a,b)=>a-b);
    return sorted[Math.floor((sorted.length-1)*q)];
  }

  async function run(ca,data,options={},progress=()=>{},shouldStop=()=>false) {
    const cfg=validateOptions(options);
    const b=makeBindings(ca,data);
    try {
    const optimizer=new Optimizer(ca,b,data,cfg.controller);
    const indi=cfg.controller==='hybrid'?new INDI(b,data):null;
    const initial=data.initial.slice();initial[2]=cfg.altitude;
    const estimator=new Estimator(b,data,initial);
    const s=data.sensors, p=data.profile;
    const imuNoise=random(cfg.seed), gpsNoise=random(cfg.seed+1), rpmNoise=random(cfg.seed+2), biasNoise=random(cfg.seed+3);
    // Independent gyro stream: both controllers see the same noise realization,
    // including their observation at t=0, without borrowing a previous-tick gyro.
    const gyroNoise=random(cfg.seed+4);
    let ba=Array.from({length:3},()=>s.accel_bias_std_mps2*biasNoise());
    let bg=Array.from({length:3},()=>s.gyro_bias_std_rad_s*biasNoise());
    let x=initial.slice(), sensedGyro, sensedRpm;
    function captureRates(){
      sensedGyro=x.slice(10,13).map((v,i)=>v+bg[i]+s.gyro_std_rad_s*gyroNoise());
      sensedRpm=x.slice(13,17).map(v=>v+s.rpm_std_rad_s*rpmNoise());
    }
    captureRates();
    if(indi) indi.gyro=cfg.feedback==='eskf'?sensedGyro.slice():x.slice(10,13);
    let u=x.slice(13,17), request=optimizer.previous.slice(), voltage=p.battery.series*4.2;
    let samples=0, sumV2=0, sumZ2=0, sumEstimate2=0, energy=0, outside=0, maxEnergyResidual=0, maxBusResidual=0;
    let failure=null, stopped=false, saturation=0, imuSamples=0, gpsSamples=0, maxRate=0;
    let currentLimited=0,voltageLimited=0,trackingLimited=0,coasting=0,allocationLimited=0,allocationThrustResidual=0,allocationAlphaResidual=0;
    const pendingGPS=[], gpsEvents=[], trace=[], ticks=Math.round(cfg.seconds/DT), begin=performance.now();
    const yieldMessages=()=>new Promise(resolve=>setTimeout(resolve,0));
    function record(t,d,dx,imu=null){
      // All channels in a row refer to x(t), not a mixture of x(t) and x(t+dt).
      trace.push({t,state:x.slice(),position_m:x.slice(0,3),v:x.slice(3,6),z:x[2],q_xyzw:x.slice(6,10),
        body_rate_rad_s:x.slice(10,13),rotor_rad_s:x.slice(13,17),command_rad_s:u.slice(),
        reference:reference(t,cfg),virtual:indi?request.slice():null,soc:x[17],
        motor:motorDiagnostics(d),alpha_rad_s2:dx.slice(10,13),
        estimate:estimator.state.slice(),covariance_diagonal:Array.from({length:15},(_,i)=>estimator.cov[i*16]),
        sensor:{capture_t_s:t,gyro_t_s:t,rotor_t_s:t,imu_t_s:imu?t:null,
          gyro_rad_s:sensedGyro.slice(),rotor_rad_s:sensedRpm.slice(),imu,
          true_accel_bias_mps2:ba.slice(),true_gyro_bias_rad_s:bg.slice()},
        allocation:indi?.last||null});
    }
    function report(t){progress({t,total:cfg.seconds,v:x.slice(3,6),z:x[2],p:x.slice(0,3),q:x.slice(6,10),rpm:x.slice(13,17),solves:optimizer.stats.length});}
    for(let tick=0;tick<ticks;tick++) {
      const t=tick*DT, env=environment(t,cfg), target=reference(t,cfg);
      // Controller receives this explicit observation. No true-state closure in it.
      const feedback=cfg.feedback==='truth'?x.slice(0,17):estimator.feedback(sensedGyro,sensedRpm);
      if(tick%PERIOD===0) {
        await yieldMessages();
        if(shouldStop()){stopped=true;break;}
        // Delta-input cost starts from the last achieved nominal allocation,
        // not an impossible requested virtual action. This is not plant truth.
        if(indi?.last)optimizer.previous=indi.last.allocated.slice();
        request=optimizer.solve(feedback,referenceHorizon(t,cfg),voltage,[0,0,0]);
        const stat=optimizer.stats.at(-1);if(stat)stat.t_s=t;
        report(t);
        await yieldMessages();
        if(shouldStop()){stopped=true;break;}
      }
      u=indi?indi.update(feedback,request,cfg.feedback==='eskf'?sensedGyro:feedback.slice(10,13)):request.slice();
      if(indi)indi.last.t_s=t;
      if(u.some(v=>v<1e-6||v>data.solvers.nmpc.max_rotor_rad_s-1e-6)) saturation++;
      const dx=b.call(b.functions.rhs,x,u,env,cfg.scales)[0];
      const d=b.call(b.functions.diag,x,u,env,cfg.scales);
      const motor=motorDiagnostics(d);
      currentLimited+=motor.current_limited.some(Boolean);voltageLimited+=motor.voltage_limited.some(Boolean);
      trackingLimited+=motor.tracking_limited.some(Boolean);coasting+=motor.coasting.some(Boolean);
      if(indi){
        allocationLimited+=indi.last.limited;
        allocationThrustResidual=Math.max(allocationThrustResidual,Math.abs(indi.last.residual[0]));
        allocationAlphaResidual=Math.max(allocationAlphaResidual,norm(indi.last.residual.slice(1)));
      }
      voltage=d[4][0]; energy+=d[6][0]*DT;
      outside+=d[13].some(v=>v!==0)?1:0;
      maxEnergyResidual=Math.max(maxEnergyResidual,Math.abs(d[11][0]));
      maxBusResidual=Math.max(maxBusResidual,Math.abs(d[12][0]));
      const specific=rotate(x.slice(6,10),add(dx.slice(3,6),[0,0,p.g]),true);
      const imu=[...specific.map((v,i)=>v+ba[i]+s.accel_std_mps2*imuNoise()),...sensedGyro];
      if(tick%(1000/cfg.log_hz)===0)record(t,d,dx,imu);
      estimator.predict(imu); imuSamples++;
      x=b.call(b.functions.step,x,u,env,cfg.scales)[0];
      ba=ba.map(v=>v+s.accel_bias_rw_mps2_sqrt_s*Math.sqrt(DT)*biasNoise());
      bg=bg.map(v=>v+s.gyro_bias_rw_rad_s_sqrt_s*Math.sqrt(DT)*biasNoise());
      captureRates();
      const nextTick=tick+1;
      if(nextTick%s.gps_period_ticks===0) {
        pendingGPS.push({capture:nextTick,arrival:nextTick+s.gps_latency_ticks,
          value:x.slice(0,6).map((v,i)=>v+(i<3?s.gps_position_std_m:s.gps_velocity_std_mps)*gpsNoise())});
        gpsSamples++;
      }
      while(pendingGPS.length && pendingGPS[0].arrival<=nextTick) {
        const gps=pendingGPS.shift(); estimator.gps(gps.capture,gps.value);gpsEvents.push(estimator.lastGPS);
      }
      samples++; sumV2+=sub(x.slice(3,6),reference(nextTick*DT,cfg).slice(0,3)).reduce((s,v)=>s+v*v,0);
      sumZ2+=(x[2]-target[3])**2;
      sumEstimate2+=sub(estimator.state.slice(0,3),x.slice(0,3)).reduce((s,v)=>s+v*v,0);
      maxRate=Math.max(maxRate,norm(x.slice(10,13)));
      if(!x.every(Number.isFinite)||x[2]<0||norm(x.slice(10,13))>100||x.slice(13,17).some(v=>v<-.01)) {
        failure='Plant became invalid / ground crossing / rate >100 rad/s'; break;
      }
      if(x[17]<p.battery.minimum_soc) {failure='Battery minimum SOC reached';break;}
    }
    const end=samples*DT,env=environment(end,cfg);
    if(x.every(Number.isFinite))record(end,b.call(b.functions.diag,x,u,env,cfg.scales),b.call(b.functions.rhs,x,u,env,cfg.scales)[0]);
    report(end);
    const fraction=v=>samples?v/samples:null,rmse=v=>samples?Math.sqrt(v/samples):null;
    const timing=optimizer.stats.map(v=>v.ms);
    return {schema_version:2,configuration:cfg,profile_id:p.id,profile_name:p.name||null,
      implementation:data.provenance||null,solver:'CasADi/IPOPT WebAssembly 3.8.0',
      simulated_seconds:samples*DT,wall_seconds:(performance.now()-begin)/1000,failure,
      status:failure?'failed':stopped?'stopped':'completed',
      event_coverage:{speed_step:cfg.scenario!=='hover'&&end>1,gust_started:cfg.scenario==='gust'&&end>2,gust_completed:cfg.scenario==='gust'&&end>=3},
      counts:{plant:samples,imu:imuSamples,nmpc:optimizer.stats.length,indi:indi?.samples||0,gps_captured:gpsSamples,
              gps_fused:estimator.updates,gps_replayed_imu:estimator.replayed},
      metrics:{velocity_rmse_mps:rmse(sumV2),altitude_rmse_m:rmse(sumZ2),
               position_estimation_rmse_m:rmse(sumEstimate2),electrical_energy_J:energy,
               saturation_fraction:fraction(saturation),outside_prop_map_fraction:fraction(outside),max_body_rate_rad_s:maxRate,
               current_limited_fraction:fraction(currentLimited),voltage_limited_fraction:fraction(voltageLimited),
               tracking_limited_fraction:fraction(trackingLimited),coasting_fraction:fraction(coasting),
               allocation_limited_fraction:indi?fraction(allocationLimited):null,
               allocation_thrust_residual_max_N:indi?allocationThrustResidual:null,
               allocation_alpha_residual_max_rad_s2:indi?allocationAlphaResidual:null,
               motor_energy_residual_W:maxEnergyResidual,bus_residual_V:maxBusResidual,
               solver_failures:optimizer.stats.filter(s=>!s.success).length,
               solve_ms_p50:percentile(timing,.5),solve_ms_p95:percentile(timing,.95),solve_ms_max:timing.length?Math.max(...timing):null,
               deadline_misses_20ms:timing.filter(t=>t>20).length},
      final:x,trace,solves:optimizer.stats,gps_events:gpsEvents,
      units:{state:'p[m] v[m/s] q[xyzw] omega[rad/s] rotor[rad/s] SOC[1]',imu:'specific force[m/s²], gyro[rad/s]',
        covariance_diagonal:'15D error: p, v, attitude(rad), accel bias, gyro bias',virtual:'T[N], angular acceleration[rad/s²]'},
      limitations:[...p.assumptions,'Simulation time is paused while IPOPT solves; wall deadline statistics are NOT a hard-real-time guarantee.',
                    'Allocation reports nominal static rotor forces plus measured angular-acceleration increments, NOT instant actuator achievement. Motor spool reaction is not explicitly allocated.',
                    'Version 2 logs use synchronized gyro/RPM timestamps and independent sensor noise streams; old and new seed traces are not identical.',
                    'GPS innovation and NIS are logged but outliers are not rejected. Contact is only a CG ground-crossing abort, not STL collision dynamics.',
                    '17-state NMPC holds measured bus voltage over its 1 s horizon. Wind is unobserved and assumed zero by both controllers.',
                    'Known initial pose/alignment; IMU Euler propagation, timestamped GPS correction and replay; GPS delay is 20 ms.',
                    'Outside-map samples use an explicit passive continuation; these are NOT validated propulsion performance results.']};
    } finally {b.dispose();}
  }
  const api={run,makeBindings,boundedIncrement,allocateThrust,motorDiagnostics,validateOptions,referenceHorizon,Estimator,Optimizer,INDI,rotate,random,reference,environment,shiftPrediction};
  if(typeof module!=='undefined'&&module.exports) module.exports=api;
  else root.ResearchRuntime=api;
})(globalThis);
