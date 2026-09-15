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
      this.previous=this.meta.hover.slice(); this.guess=null; this.stats=[];
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
      return this.previous.slice();
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
      let [state,cov]=this.b.call(this.b.estimator.gps,old[0],old[1],measurement);
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
    }
    update(state, desired, rateMeasurement=state.slice(10,13)) {
      // Differentiate the measured gyro, not the bias-corrected state: a GPS
      // update to estimated gyro bias must not create a fictitious alpha impulse.
      const rpm=state.slice(13,17), gyro=rateMeasurement;
      const axial=rotate(state.slice(6,10),state.slice(3,6),true)[0];
      const [raw]=this.b.call(this.b.functions.effect,rpm,[axial]);
      const midpoint=rpm.map((v,i)=>(v+this.nRaw[i])/2);
      this.n=this.n.map((v,i)=>v+this.lpf*(midpoint[i]-v));
      this.alpha=this.alpha.map((v,i)=>v+this.lpf*((gyro[i]-this.gyro[i])/DT-v));
      this.thrust+=this.lpf*((raw[0]+this.thrustRaw)/2-this.thrust);
      this.thrustRaw=raw[0];
      this.gyro=gyro.slice(); this.nRaw=rpm.slice();
      const [,flat]=this.b.call(this.b.functions.effect,this.n,[axial]);
      const g=Array.from({length:4},(_,r)=>Array.from({length:4},(_,c)=>flat[c*4+r]));
      const error=sub(desired,[this.thrust,...this.alpha]);
      const delta=boundedIncrement(g,error,this.n.map(n=>-n),this.n.map(n=>this.max-n),
                                   [1/(this.p.mass_kg*this.p.g),.01,.01,.01]);
      const cmd=add(this.n,delta).map(v=>clamp(v,0,this.max));
      this.samples++; if(cmd.some(v=>v<1e-6||v>this.max-1e-6)) this.saturated++;
      return cmd;
    }
  }

  function reference(t, config) {
    // This changes only the REFERENCE. The actual velocity is always integrated.
    const vx=config.scenario==='hover'?0:(t<1?0:config.speed);
    return [vx,0,0,20];
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
    const cfg={controller:'hybrid',feedback:'truth',scenario:'step',seconds:4,speed:3,seed:42,
               scales:[1,1,1,1,1],...options};
    if(!['hybrid','nmpc'].includes(cfg.controller)) throw new Error('Unknown controller');
    if(!['truth','eskf'].includes(cfg.feedback)) throw new Error('Unknown feedback');
    if(!['hover','step','gust'].includes(cfg.scenario)) throw new Error('Unknown scenario');
    if(!Number.isFinite(cfg.seconds)||cfg.seconds<=0||cfg.seconds>120) throw new Error('Duration must be 0–120 seconds');
    if(!Number.isFinite(cfg.speed)||cfg.speed<0||cfg.speed>100) throw new Error('Reference speed must be 0–100 m/s');
    if(cfg.scales.length!==5||cfg.scales.some(v=>!Number.isFinite(v)||v<.5||v>1.5)) throw new Error('Invalid model-error scales');
    const b=makeBindings(ca,data);
    try {
    const optimizer=new Optimizer(ca,b,data,cfg.controller);
    const indi=cfg.controller==='hybrid'?new INDI(b,data):null;
    const estimator=new Estimator(b,data,data.initial);
    const s=data.sensors, p=data.profile;
    const imuNoise=random(cfg.seed), gpsNoise=random(cfg.seed+1), rpmNoise=random(cfg.seed+2), biasNoise=random(cfg.seed+3);
    let ba=Array.from({length:3},()=>s.accel_bias_std_mps2*biasNoise());
    let bg=Array.from({length:3},()=>s.gyro_bias_std_rad_s*biasNoise());
    let x=data.initial.slice(), sensedGyro=bg.slice(), sensedRpm=x.slice(13,17);
    if(indi) indi.gyro=cfg.feedback==='eskf'?sensedGyro.slice():x.slice(10,13);
    let u=x.slice(13,17), request=optimizer.previous.slice(), voltage=p.battery.series*4.2;
    let samples=0, sumV2=0, sumZ2=0, sumEstimate2=0, energy=0, outside=0, maxEnergyResidual=0, maxBusResidual=0;
    let failure=null, saturation=0, imuSamples=0, gpsSamples=0, maxRate=0;
    const pendingGPS=[], trace=[], ticks=Math.round(cfg.seconds/DT), begin=performance.now();
    for(let tick=0;tick<ticks;tick++) {
      const t=tick*DT, env=environment(t,cfg), target=reference(t,cfg);
      // Controller receives this explicit observation. No true-state closure in it.
      const feedback=cfg.feedback==='truth'?x.slice(0,17):estimator.feedback(sensedGyro,sensedRpm);
      if(tick%PERIOD===0) {
        const refs=Array.from({length:21},(_,k)=>reference(t+k*.05,cfg));
        request=optimizer.solve(feedback,refs,voltage,[0,0,0]);
      }
      u=indi?indi.update(feedback,request,cfg.feedback==='eskf'?sensedGyro:feedback.slice(10,13)):request.slice();
      if(u.some(v=>v<1e-6||v>data.solvers.nmpc.max_rotor_rad_s-1e-6)) saturation++;
      const dx=b.call(b.functions.rhs,x,u,env,cfg.scales)[0];
      const d=b.call(b.functions.diag,x,u,env,cfg.scales);
      voltage=d[4][0]; energy+=d[6][0]*DT;
      outside+=d[13].some(v=>v!==0)?1:0;
      maxEnergyResidual=Math.max(maxEnergyResidual,Math.abs(d[11][0]));
      maxBusResidual=Math.max(maxBusResidual,Math.abs(d[12][0]));
      const specific=rotate(x.slice(6,10),add(dx.slice(3,6),[0,0,p.g]),true);
      const imu=[...specific.map((v,i)=>v+ba[i]+s.accel_std_mps2*imuNoise()),
                 ...x.slice(10,13).map((v,i)=>v+bg[i]+s.gyro_std_rad_s*imuNoise())];
      estimator.predict(imu); imuSamples++;
      sensedGyro=imu.slice(3);
      x=b.call(b.functions.step,x,u,env,cfg.scales)[0];
      sensedRpm=x.slice(13,17).map(v=>v+s.rpm_std_rad_s*rpmNoise());
      ba=ba.map(v=>v+s.accel_bias_rw_mps2_sqrt_s*Math.sqrt(DT)*biasNoise());
      bg=bg.map(v=>v+s.gyro_bias_rw_rad_s_sqrt_s*Math.sqrt(DT)*biasNoise());
      const nextTick=tick+1;
      if(nextTick%s.gps_period_ticks===0) {
        pendingGPS.push({capture:nextTick,arrival:nextTick+s.gps_latency_ticks,
          value:x.slice(0,6).map((v,i)=>v+(i<3?s.gps_position_std_m:s.gps_velocity_std_mps)*gpsNoise())});
        gpsSamples++;
      }
      while(pendingGPS.length && pendingGPS[0].arrival<=nextTick) {
        const gps=pendingGPS.shift(); estimator.gps(gps.capture,gps.value);
      }
      samples++; sumV2+=sub(x.slice(3,6),reference(nextTick*DT,cfg).slice(0,3)).reduce((s,v)=>s+v*v,0);
      sumZ2+=(x[2]-target[3])**2;
      sumEstimate2+=sub(estimator.state.slice(0,3),x.slice(0,3)).reduce((s,v)=>s+v*v,0);
      maxRate=Math.max(maxRate,norm(x.slice(10,13)));
      if(nextTick%20===0) trace.push({t:nextTick*DT,v:x.slice(3,6),z:x[2],reference:reference(nextTick*DT,cfg),
        rpm:x.slice(13,17),command:u.slice(),virtual:indi?request.slice():null,soc:x[17],voltage,
        estimate:estimator.state.slice(0,6),alpha:dx.slice(10,13),power:d[6][0]});
      if(!x.every(Number.isFinite)||x[2]<0||norm(x.slice(10,13))>100||x.slice(13,17).some(v=>v<-.01)) {
        failure='Plant became invalid / ground crossing / rate >100 rad/s'; break;
      }
      if(x[17]<p.battery.minimum_soc) {failure='Battery minimum SOC reached';break;}
      if(nextTick%100===0) {
        progress({t:nextTick*DT,total:cfg.seconds,v:x.slice(3,6),z:x[2],q:x.slice(6,10),rpm:x.slice(13,17),solves:optimizer.stats.length});
        // Yield the worker event loop so Stop messages are serviced between solves.
        await new Promise(resolve=>setTimeout(resolve,0));
        if(shouldStop()) {failure='User stopped';break;}
      }
    }
    const timing=optimizer.stats.map(v=>v.ms);
    return {configuration:cfg,profile_id:p.id,implementation:data.provenance||null,solver:'CasADi/IPOPT WebAssembly 3.8.0',
      simulated_seconds:samples*DT,wall_seconds:(performance.now()-begin)/1000,failure,
      counts:{plant:samples,imu:imuSamples,nmpc:optimizer.stats.length,indi:indi?.samples||0,gps_captured:gpsSamples,
              gps_fused:estimator.updates,gps_replayed_imu:estimator.replayed},
      metrics:{velocity_rmse_mps:Math.sqrt(sumV2/samples),altitude_rmse_m:Math.sqrt(sumZ2/samples),
               position_estimation_rmse_m:Math.sqrt(sumEstimate2/samples),electrical_energy_J:energy,
               saturation_fraction:saturation/samples,outside_prop_map_fraction:outside/samples,max_body_rate_rad_s:maxRate,
               motor_energy_residual_W:maxEnergyResidual,bus_residual_V:maxBusResidual,
               solver_failures:optimizer.stats.filter(s=>!s.success).length,
               solve_ms_p50:percentile(timing,.5),solve_ms_p95:percentile(timing,.95),solve_ms_max:Math.max(...timing),
               deadline_misses_20ms:timing.filter(t=>t>20).length},
      final:x,trace,solves:optimizer.stats,
      limitations:[...p.assumptions,'Simulation time is paused while IPOPT solves; wall deadline statistics are NOT a hard-real-time guarantee.',
                    '17-state NMPC holds measured bus voltage over its 1 s horizon. Wind is unobserved and assumed zero by both controllers.',
                    'Known initial pose/alignment; IMU Euler propagation, timestamped GPS correction and replay; GPS delay is 20 ms.',
                    'Outside-map samples use an explicit passive continuation; these are NOT validated propulsion performance results.']};
    } finally {b.dispose();}
  }
  const api={run,makeBindings,boundedIncrement,Estimator,Optimizer,INDI,rotate,random,reference,environment,shiftPrediction};
  if(typeof module!=='undefined'&&module.exports) module.exports=api;
  else root.ResearchRuntime=api;
})(globalThis);
