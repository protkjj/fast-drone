/* Shared Node / Web Worker experiment runner. All dynamics come from model.py. */
(function (root) {
  'use strict';
  const DT = .001, PERIOD = 20;
  const clamp = (x, a, b) => Math.min(b, Math.max(a, x));
  const norm = x => Math.hypot(...x);
  const add = (a, b) => a.map((v, i) => v+b[i]);
  const sub = (a, b) => a.map((v, i) => v-b[i]);
  const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  // 논문 식(43): 0<=u<=1에서 5-9차 다항식, 양 끝에서 1-4차 도함수가 0이라
  // 부드럽게 이어진다. 구간 밖은 0/1로 연장. "계단형 속도 명령은 입력
  // 포화·지연 진단 전용이고 주 기동 비교엔 연속 참조를 쓴다"(§5.5)는
  // 논문 자신의 규칙을 반영한 참조 생성기 -- 'step' 시나리오는 그 진단
  // 용도로 남겨두고 새 'ramp' 시나리오가 이 매끄러운 참조를 쓴다.
  function smoothstep(u) {
    if (u <= 0) return 0;
    if (u >= 1) return 1;
    const u2=u*u, u3=u2*u, u4=u3*u, u5=u4*u;
    return 126*u5 - 420*u5*u + 540*u5*u2 - 315*u5*u3 + 70*u5*u4;
  }
  const dot3 = (a,b) => a.reduce((s,v,i)=>s+v*b[i],0);
  function quatMultiply(a, b) {
    // Hamilton product a (x) b, scalar-last [x,y,z,w].
    return [a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
            a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
            a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
            a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]];
  }
  // Exact SO(3) logarithm of the relative rotation, not the small-angle
  // antisymmetric-part shortcut used elsewhere in this file (ObservationController).
  // Paper 식(37)-(39) calls this "국소 회전벡터" (local rotation vector) of R^T R_d.
  // current/desired: each an array of 3 world-frame column vectors (body axes).
  function relativeRotationVector(current, desired) {
    const R = current.map(ci => desired.map(dj => dot3(ci, dj))); // R = current^T . desired
    const trace = R[0][0]+R[1][1]+R[2][2];
    const angle = Math.acos(clamp((trace-1)/2, -1, 1));
    const raw = [R[2][1]-R[1][2], R[0][2]-R[2][0], R[1][0]-R[0][1]];
    if (angle < 1e-6) return raw.map(v => v/2);
    const s = Math.sin(angle);
    if (s < 1e-6) {
      // Near-180 deg fallback: axis magnitude from the diagonal, sign left
      // unresolved (ambiguous at this singularity). Only keeps the loop
      // finite; this operating point is already a control failure elsewhere.
      const axis = [Math.sqrt(Math.max(0,(R[0][0]+1)/2)), Math.sqrt(Math.max(0,(R[1][1]+1)/2)),
                    Math.sqrt(Math.max(0,(R[2][2]+1)/2))];
      return axis.map(v => v*angle);
    }
    return raw.map(v => v*angle/(2*s));
  }
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
  // Same LCG as random(), but returns the raw uniform draw -- for GPS dropout
  // (표7), not a noise magnitude.
  function uniformRandom(seed) {
    let state=seed>>>0;
    return () => { state=(Math.imul(1664525,state)+1013904223)>>>0; return (state+.5)/4294967296; };
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

  const numericCache=new WeakMap();
  function makeBindings(ca, data, solverKinds=Object.keys(data.solvers),useNumeric=true) {
    const decode=group=>Object.fromEntries(Object.entries(group).map(([k,v])=>[k,ca.Function.deserialize(v)]));
    const functions=decode(data.functions), estimator=decode(data.estimator);
    const solvers=Object.fromEntries(solverKinds.map(k=>[k,ca.Function.deserialize(data.solvers[k].function)]));
    const numeric=new Map();
    if(useNumeric&&data.numeric){
      if(data.numeric.format!=='casadi-scalar-js-v1')throw new Error('Unknown numeric graph format');
      if(!numericCache.has(data))numericCache.set(data,new Function(data.numeric.source)());
      const compiled=numericCache.get(data);
      for(const [key,layout] of Object.entries(data.numeric.layout)){
        const [group,name]=key.split('.'),fn=(group==='functions'?functions:estimator)[name];
        numeric.set(fn,args=>{
          const flat=compiled[key](layout.inputs.flatMap((indices,i)=>indices.map(j=>args[i][j])));
          let offset=0;
          return layout.outputs.map(({size,indices})=>{
            const values=Array(size).fill(0);for(const index of indices)values[index]=flat[offset++];return values;
          });
        });
      }
    }
    // elements() includes structural zeros, unlike nonzeros(). Matrix values
    // returned through the 3.8.0 binding are indexed Proxies. Calling delete()
    // on them does not unregister the original finalizer token and can cause
    // a double free at GC. Let the package's FinalizationRegistry own matrices.
    const array=dm=>dm.elements();
    const call=(fn,...args)=>numeric.has(fn)?numeric.get(fn)(args):fn.call(args.map(v=>ca.DM(v))).map(array);
    const dispose=()=>[solvers,functions,estimator].forEach(group=>Object.values(group).forEach(fn=>fn.delete()));
    return {functions,estimator,solvers,call,array,dispose};
  }

  // Observation controller, explicitly NOT NMPC. It only sees the same feedback
  // state as other controllers. Bounded virtual actions pass through the SAME
  // 1 kHz INDI and physical actuator model; no state is assigned to a target.
  class ObservationController {
    constructor(binding,data){
      this.b=binding;this.p=data.profile;this.stats=[];
      this.previous=[this.p.mass_kg*this.p.g,0,0,0];
    }
    solve(state,refs){
      const p=this.p,q=state.slice(6,10),ref=refs[0],v=state.slice(3,6);
      const acceleration=[clamp(1.4*(ref[0]-v[0]),-6,6),clamp(-1.8*v[1],-6,6),
        clamp(4*(ref[3]-state[2])-3*v[2],-6,6)];
      const [aero]=this.b.call(this.b.functions.aero,state.slice(0,13),[0,0,0]);
      const force=sub(acceleration.map((v,i)=>p.mass_kg*(v+(i===2?p.g:0))),rotate(q,aero));
      const unit=v=>v.map(x=>x/Math.max(1e-12,norm(v))),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
      const desiredX=unit(force),heading=[0,1,0];
      const desiredY=unit(sub(heading,desiredX.map(v=>v*dot(heading,desiredX))));
      const desired=[desiredX,desiredY,cross(desiredX,desiredY)];
      const current=[[1,0,0],[0,1,0],[0,0,1]].map(axis=>rotate(q,axis));
      const errorWorld=current.map((axis,i)=>cross(axis,desired[i])).reduce(add,[0,0,0]).map(v=>v*.5);
      const error=rotate(q,errorWorld,true),rate=state.slice(10,13);
      const alpha=error.map((v,i)=>clamp([60,45,45][i]*v-[12,11,11][i]*rate[i],-80,80));
      this.previous=[Math.max(0,dot(force,current[0])),...alpha];
      return this.previous.slice();
    }
  }

  // Cascaded PID baseline (표5 CPID, 식37-39). Thrust axis is body +x, so
  // R_d's FIRST column (not the third, unlike a conventional z-down quad)
  // equals F_d/||F_d||. Static constrained allocation only -- deliberately
  // no INDI feedback (paper: "주 CPID는... INDI 각가속도 피드백은 사용하지 않는다").
  // Gains below are placeholders at the same order of magnitude as this
  // repo's other baseline controllers (control/controller.py, ObservationController);
  // the paper's own protocol requires an independent tuning pass before any
  // comparison is reported, not a specific numeric target.
  // Shared outer loop for every "explicit attitude reference" baseline (표5
  // CPID and GINDI both build R_d this way -- 식37-39). Thrust axis is body
  // +x, so R_d's FIRST column equals F_d/||F_d|| (not the third column, the
  // way a conventional z-down quad would). Returns {Td,Md}; Md is a MOMENT
  // (Nm), not yet divided by inertia -- callers decide how to use it (CPID:
  // static allocateThrust; GINDI: divide by inertia and hand to shared INDI).
  // mem carries the integrator/filter/heading-hold state across calls.
  function geometricAttitudeCommand(binding, mem, gains, p, maxRotorRadS, state, refs, wind) {
    const dt = PERIOD*DT, q = state.slice(6, 10), v = state.slice(3, 6), z = state[2];
    const ref = refs[0];
    const vd = [ref[0], ref[1], gains.kz*(ref[3]-z)];
    const ev = sub(vd, v);
    if (!mem.lastEv) mem.lastEv = ev.slice();
    const alpha = 1-Math.exp(-2*Math.PI*10*dt);
    mem.filteredDerivV = mem.filteredDerivV.map((x, i) => x+alpha*((ev[i]-mem.lastEv[i])/dt-x));
    mem.lastEv = ev.slice();
    const ad = ev.map((e, i) => gains.kpV[i]*e+gains.kiV[i]*mem.integralV[i]+gains.kdV[i]*mem.filteredDerivV[i]);
    const [aero] = binding.call(binding.functions.aero, state.slice(0, 13), wind);
    const worldAero = rotate(q, aero);
    const Fd = sub([p.mass_kg*ad[0], p.mass_kg*ad[1], p.mass_kg*(ad[2]+p.g)], worldAero);
    const Fnorm = norm(Fd);
    let ex, ey, ez;
    if (Fnorm < 1e-6 && mem.desiredAxes) {
      [ex, ey, ez] = mem.desiredAxes;
    } else {
      ex = Fnorm < 1e-6 ? [1, 0, 0] : Fd.map(x => x/Fnorm);
      const heading = [0, 1, 0], proj = sub(heading, ex.map(x => x*dot3(heading, ex)));
      const pn = norm(proj);
      ey = (pn > 1e-6 ? proj.map(x => x/pn) : cross([0, 0, 1], ex));
      const eyn = Math.max(norm(ey), 1e-9); ey = ey.map(x => x/eyn);
      ez = cross(ex, ey);
    }
    mem.desiredAxes = [ex, ey, ez];
    const Td = Fnorm;
    const current = [[1, 0, 0], [0, 1, 0], [0, 0, 1]].map(axis => rotate(q, axis));
    const eR = relativeRotationVector(current, [ex, ey, ez]);
    const omega = state.slice(10, 13);
    const omegaD = eR.map(x => gains.kR*x);
    const eOmega = sub(omegaD, omega);
    const Md = eOmega.map((e, i) => gains.kpW*e+gains.kiW*mem.integralW[i]);
    const axial = rotate(q, v, true)[0];
    const [caps] = binding.call(binding.functions.rotors, Array(4).fill(maxRotorRadS), [axial]);
    const capacity = caps.reduce((s, x) => s+x, 0);
    // Conditional integration: hold both integrators while thrust-saturated
    // (documented anti-windup choice, per paper 식37 commentary). Shared by
    // CPID and GINDI since both track the SAME Td/Md reference.
    if (Td <= capacity+1e-9) {
      mem.integralV = mem.integralV.map((x, i) => x+ev[i]*dt);
      mem.integralW = mem.integralW.map((x, i) => x+eOmega[i]*dt);
    }
    return {Td, Md, axial, caps, capacity};
  }
  function newGuidanceMemory() {
    return {integralV: [0, 0, 0], integralW: [0, 0, 0], filteredDerivV: [0, 0, 0],
            lastEv: null, desiredAxes: null};
  }
  // Placeholder gains, same order of magnitude as this repo's other baseline
  // controllers -- the paper's protocol requires an independent tuning pass
  // per controller, not a shared number. CPID and GINDI genuinely need
  // DIFFERENT attitude-rate gains here: sharing CPID's kR/kpW/kiW with GINDI
  // diverges by ~9s of a 3 m/s step (checked directly, not assumed) because
  // INDI tracks the commanded [T,alpha] much tighter than the static
  // allocator, so the same outer-loop gains are effectively higher-bandwidth
  // through GINDI. GINDI's own values below were empirically checked stable
  // over 15s; neither set claims to be tuned.
  const CPID_GAINS = {kpV: [1.4, 1.4, 4.0], kiV: [.15, .15, .6], kdV: [.1, .1, .2],
                      kz: 1.8, kR: 8.0, kpW: 45.0, kiW: 5.0};
  const GINDI_GAINS = {kpV: [1.4, 1.4, 4.0], kiV: [.15, .15, .6], kdV: [.1, .1, .2],
                       kz: 1.8, kR: 3.0, kpW: 15.0, kiW: 2.0};

  class CPID {
    constructor(binding, data) {
      this.b = binding; this.p = data.profile;
      this.gains = CPID_GAINS; this.mem = newGuidanceMemory();
      this.max = data.solvers.hybrid.max_rotor_rad_s; this.stats = [];
      // Fixed (non-measured) nominal effectiveness at the hover rotor speed --
      // "정적 배분": the matrix does not re-linearize around the current
      // measurement the way INDI does. The thrust ceiling in geometricAttitudeCommand
      // still uses the CURRENT axial inflow, since that bound is a physical
      // fact, not a control-law choice.
      const a = this.p.arm_m/Math.sqrt(2), inertia = this.p.inertia_kg_m2;
      this.dirs = [1, -1, 1, -1]; this.ys = [a, -a, -a, a]; this.zs = [a, a, -a, -a];
      const nHov = data.initial.slice(13, 17);
      const [, , dT, dQ] = binding.call(binding.functions.rotors, nHov, [0]);
      this.matrix = [this.dirs.map((d, i) => d*dQ[i]/Math.max(dT[i], 1e-9)/inertia[0]),
                    this.zs.map(v => v/inertia[1]), this.ys.map(v => -v/inertia[2])];
      this.previous = nHov.slice(); // rotor-speed command, not a [T,alpha] virtual input
    }
    solve(state, refs, voltage, wind) {
      const {Td, Md, axial, caps, capacity} =
        geometricAttitudeCommand(this.b, this.mem, this.gains, this.p, this.max, state, refs, wind);
      const total = clamp(Td, 0, capacity);
      const target = Md.map((m, i) => m/this.p.inertia_kg_m2[i]);
      const anchor = caps.map(c => capacity > 0 ? total*c/capacity : 0);
      const force = allocateThrust(total, target, this.matrix, caps, anchor);
      const [n] = this.b.call(this.b.functions.inverse_thrust, force, [axial], [this.max]);
      return n.map(x => clamp(x, 0, this.max));
    }
  }

  // GINDI (표5): same outer geometric loop as CPID (식37-39), but the inner
  // loop is the SAME measured-feedback INDI allocation as V13/F13, not a
  // static one -- "DFBC 또는 기하학적 외부 제어와 공통 INDI". The caller
  // (run()) owns the shared INDI instance and calls indi.update() with this
  // class's [T,alpha] output, exactly as it does for hybrid/f13.
  class GINDI {
    constructor(binding, data) {
      this.b = binding; this.p = data.profile;
      this.gains = GINDI_GAINS; this.mem = newGuidanceMemory();
      this.max = data.solvers.hybrid.max_rotor_rad_s; this.stats = [];
      this.previous = [this.p.mass_kg*this.p.g, 0, 0, 0];
    }
    solve(state, refs, voltage, wind) {
      const {Td, Md} = geometricAttitudeCommand(this.b, this.mem, this.gains, this.p, this.max, state, refs, wind);
      this.previous = [Td, ...Md.map((m, i) => m/this.p.inertia_kg_m2[i])];
      return this.previous.slice();
    }
  }

  // F13 (표5): wraps the rotor-thrust-output NMPC (research/nmpc.py kind="f13",
  // research/model.py functions["f13"]) so the SAME shared INDI as
  // hybrid/gindi can consume it. The NMPC itself already predicts real
  // rigid-body rotation (J*omega_dot = M - omega x J*omega), unlike hybrid's
  // idealized omega_dot=nu -- that prediction-model difference, not the
  // allocation, is what distinguishes F13 from V13 (표5 "가장 가까운 인터페이스 비교").
  // Reaction torque uses a fixed nominal dQ/dT ratio at hover, matching
  // model.py's f13 prediction model and control/nmpc_f13.py's simplification.
  class F13Adapter {
    constructor(ca, binding, data) {
      this.inner = new Optimizer(ca, binding, data, 'f13');
      this.b = binding; this.p = data.profile; this.stats = this.inner.stats;
      const a = this.p.arm_m/Math.sqrt(2);
      this.dirs = [1, -1, 1, -1]; this.ys = [a, -a, -a, a]; this.zs = [a, a, -a, -a];
      const nHov = data.initial.slice(13, 17);
      const [, , dT, dQ] = binding.call(binding.functions.rotors, nHov, [0]);
      this.kRatio = dQ.map((q, i) => q/Math.max(dT[i], 1e-9));
      this.previous = [this.p.mass_kg*this.p.g, 0, 0, 0];
    }
    solve(...args) {
      const f = this.inner.solve(...args); // rotor thrust, 4D (N)
      const torque = f.map((fi, i) => this.kRatio[i]*fi);
      const T = f.reduce((s, v) => s+v, 0);
      const inertia = this.p.inertia_kg_m2;
      const Mx = this.dirs.reduce((s, d, i) => s+d*torque[i], 0)/inertia[0];
      const My = this.zs.reduce((s, z, i) => s+z*f[i], 0)/inertia[1];
      const Mz = -this.ys.reduce((s, y, i) => s+y*f[i], 0)/inertia[2];
      this.previous = [T, Mx, My, Mz];
      return this.previous.slice();
    }
  }

  // Gain-scheduled LQR baseline (표5 GSLQR, 식40-42). K_j precomputed offline
  // (research/gain_schedule.py) at level-flight trims of the SAME selected
  // profile and linearly interpolated by speed here. The schedule only
  // covers 0-18 m/s: research/TRIM_ENVELOPE_AUDIT.md and a direct 2 m/s scan
  // (2026-09-22) found NO level-flight trim for this airframe from 20-80 m/s
  // (negative rotor thrust required) and current-limited trims at 82+ m/s.
  // Outside the table this class clamps to the nearest edge gain/trim --
  // it does not claim validity there.
  class GSLQR {
    constructor(binding, data) {
      this.b = binding; this.p = data.profile;
      this.schedule = data.gslqr;
      if (!this.schedule || !this.schedule.speeds_mps.length)
        throw new Error('GSLQR gain schedule missing from bundle (research/gain_schedule.py)');
      this.max = data.solvers.hybrid.max_rotor_rad_s;
      this.previous = data.initial.slice(13, 17);
      this.stats = [];
    }
    _interpolate(speed) {
      const V = this.schedule.speeds_mps, target = clamp(speed, V[0], V[V.length-1]);
      let i = 0; while (i < V.length-2 && V[i+1] < target) i++;
      const t = (target-V[i])/((V[i+1]-V[i]) || 1);
      const lerp = (a, b) => a.map((x, k) => x+t*(b[k]-x));
      return {K: this.schedule.K_r[i].map((row, r) => lerp(row, this.schedule.K_r[i+1][r])),
              xTrim: lerp(this.schedule.x_trim[i], this.schedule.x_trim[i+1]),
              uTrim: lerp(this.schedule.u_trim[i], this.schedule.u_trim[i+1])};
    }
    solve(state, refs) {
      const ref = refs[0];
      const {K, xTrim, uTrim} = this._interpolate(ref[0]);
      xTrim[2] = ref[3];
      const dz = state[2]-xTrim[2], dv = sub(state.slice(3, 6), xTrim.slice(3, 6));
      const q = state.slice(6, 10), qt = xTrim.slice(6, 10);
      let dq = quatMultiply([-qt[0], -qt[1], -qt[2], qt[3]], q);
      if (dq[3] < 0) dq = dq.map(x => -x);
      const dphi = dq.slice(0, 3).map(x => 2*x);
      const dw = sub(state.slice(10, 13), xTrim.slice(10, 13));
      const dn = sub(state.slice(13, 17), xTrim.slice(13, 17));
      const dx = [dz, ...dv, ...dphi, ...dw, ...dn];
      const u = uTrim.map((u0, i) => u0-dot3(K[i], dx));
      return u.map(x => clamp(x, 0, this.max));
    }
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
    solve(state, refs, voltage, wind, envelope=null) {
      const m=this.meta, x=state.slice(0,m.nx);
      if(!this.guess) {
        const scaledX=x.map((v,i)=>v/this.stateScale[i]);
        const scaledU=m.hover.map((v,i)=>v/this.commandScale[i]);
        this.guess=[...Array.from({length:m.N+1},()=>scaledX).flat(),...Array.from({length:m.N},()=>scaledU).flat()];
      }
      const parameters=[...x,...refs.flat(),...this.previous,...wind,voltage];
      const lbg=m.lbg.slice(),ubg=m.ubg.slice(),interfaceMeta=m.hybrid_envelope;
      if(interfaceMeta){
        if(envelope?.available){
          parameters.push(...envelope.inverse.flat(),...envelope.offset);
          for(let i=0;i<4;i++){
            lbg[interfaceMeta.constraint_start+i]=envelope.force_lower_N[i]/interfaceMeta.force_scale_N;
            ubg[interfaceMeta.constraint_start+i]=envelope.force_upper_N[i]/interfaceMeta.force_scale_N;
          }
        }else parameters.push(...Array(interfaceMeta.parameter_size).fill(0));
      }
      const begin=performance.now();
      let result, status, values, residual=Infinity, success=false;
      try {
        const input={x0:this.guess,p:parameters,lbx:m.lbx,ubx:m.ubx,lbg,ubg};
        if(this.lamX) {input.lam_x0=this.lamX; input.lam_g0=this.lamG;}
        const packed=Object.fromEntries(Object.entries(input).map(([k,v])=>[k,this.ca.DM(v)]));
        result=this.solver.call(packed);
        values=this.binding.array(result.x);
        // Keep the pre-normalization acceptance gate in physical units. Scaling
        // must not disguise a large motor-state defect as a small dimensionless one.
        residual=Math.max(0,...this.binding.array(result.g).map((v,i)=>
          Math.max(lbg[i]-v,v-ubg[i])*this.constraintScale[i]));
        status=this.solver.stats();
        success=status.success && residual<1e-3 && values.every(Number.isFinite);
      } catch(error) {status={return_status:String(error.message||error),iter_count:0};}
      const elapsed=performance.now()-begin;
      this.stats.push({ms:elapsed,success,status:status.return_status,iterations:status.iter_count,residual,
        ...(interfaceMeta?{actuator_envelope:envelope||{available:false,reason:'disabled'}}:{})});
      if(success) {
        const shift=m.control_dt_s/m.prediction_dt_s;
        const lamX=this.binding.array(result.lam_x),lamG=this.binding.array(result.lam_g);
        this.lamX=[...shiftPrediction(lamX.slice(0,m.state_count),m.nx,shift),
          ...shiftPrediction(lamX.slice(m.state_count),4,shift)];
        this.lamG=[...shiftPrediction(lamG.slice(0,m.state_count),m.nx,shift),
          // This local map is rebuilt each solve; it is NOT a horizon trajectory
          // whose four multipliers can be shifted as 160-wide motor constraints.
          ...(interfaceMeta?Array(4).fill(0):shiftPrediction(lamG.slice(m.state_count),160,shift))];
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
      if(interfaceMeta&&envelope?.available){
        const force=envelope.inverse.map(row=>row.reduce((s,v,i)=>s+v*(this.command[i]-envelope.offset[i]),0));
        const violation=Math.max(0,...force.map((v,i)=>Math.max(envelope.force_lower_N[i]-v,v-envelope.force_upper_N[i])));
        Object.assign(this.stats.at(-1),{envelope_force_N:force,envelope_violation_N:violation});
      }
      // Declared fallback: hold the last bounded command; no hidden PD/INDI for NMPC.
      return this.command.slice();
    }
  }

  class Estimator {
    constructor(binding, data, initial, historyTicks=200) {
      this.b=binding; this.config=data.sensors; this.historyTicks=historyTicks;
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
      const oldest=this.tick-this.historyTicks;
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
    envelope(state,voltage){
      // Capability information flows UP; rotor commands still belong to update().
      // Only feedback RPM/velocity/attitude and measured bus voltage are used.
      // Freeze inflow and voltage for 20 x 1 ms nominal motor integration steps.
      const dt=PERIOD*DT,axial=rotate(state.slice(6,10),state.slice(3,6),true)[0];
      if(!Number.isFinite(voltage)||voltage<=0||!state.every(Number.isFinite))
        throw new Error('Invalid measured state/voltage for actuator envelope');
      const initial=state.slice(13,17).map(n=>Math.max(0,n));
      let low=initial.slice(),high=initial.slice();
      for(let k=0;k<PERIOD;k++){
        [low]=this.b.call(this.b.functions.motor_step,low,[0,0,0,0],[axial],[voltage]);
        [high]=this.b.call(this.b.functions.motor_step,high,Array(4).fill(this.max),[axial],[voltage]);
      }
      const [lo,ql]=this.b.call(this.b.functions.rotors,low,[axial]);
      const [hi,qh]=this.b.call(this.b.functions.rotors,high,[axial]);
      const details={horizon_s:dt,voltage_V:voltage,axial_mps:axial,
        rotor_initial_rad_s:initial,rotor_lower_rad_s:low,rotor_upper_rad_s:high,
        force_lower_N:lo,force_upper_N:hi};
      if([...low,...high,...lo,...hi].some(v=>!Number.isFinite(v)||v<0)||lo.some((v,i)=>v>hi[i]+1e-9))
        return {...details,available:false,reason:'invalid nominal motor interval'};
      const inertia=this.p.inertia_kg_m2,a=this.p.arm_m/Math.sqrt(2),dirs=[1,-1,1,-1];
      const ys=[a,-a,-a,a],zs=[a,a,-a,-a],dot=(x,y)=>x.reduce((s,v,i)=>s+v*y[i],0);
      // A secant in force coordinates preserves thrust/pitch/yaw exactly for the
      // frozen geometry. Roll torque is locally approximated; do NOT call this
      // a certified nonlinear reachable set, especially outside the prop map.
      const middle=low.map((v,i)=>(v+high[i])/2),[fm,qm,df,dq]=this.b.call(this.b.functions.rotors,middle,[axial]);
      // A voltage below back-EMF can leave only passive coasting. Preserve that
      // zero-width force bound instead of silently removing the constraint.
      if(hi.some((v,i)=>v-lo[i]<1e-9&&df[i]<=1e-10))
        return {...details,available:false,reason:'degenerate force effectiveness outside the local map'};
      const slope=hi.map((v,i)=>v-lo[i]>=1e-9?(qh[i]-ql[i])/(v-lo[i]):dq[i]/df[i]);
      const intercept=ql.map((v,i)=>v-slope[i]*lo[i]);
      const matrix=[Array(4).fill(1),slope.map((v,i)=>dirs[i]*v/inertia[0]),
        zs.map(v=>v/inertia[1]),ys.map(v=>-v/inertia[2])];
      const columns=Array.from({length:4},(_,i)=>linearSolve(matrix,Array.from({length:4},(_,j)=>+(i===j))));
      if(columns.some(v=>!v||v.some(x=>!Number.isFinite(x))))
        return {...details,available:false,reason:'singular virtual effectiveness'};
      const inverse=Array.from({length:4},(_,i)=>columns.map(column=>column[i]));
      const baseline=[dot(dirs,this.torque)/inertia[0],dot(zs,this.force)/inertia[1],-dot(ys,this.force)/inertia[2]];
      const offset=[0,...sub(this.alpha,baseline)];
      offset[1]+=dot(dirs,intercept)/inertia[0];
      // Log a diagnostic of roll-map curvature, not a claim of an error bound.
      const midpointError=qm.reduce((s,v,i)=>s+Math.abs(v-slope[i]*fm[i]-intercept[i])/inertia[0],0);
      return {...details,available:true,matrix,inverse,offset,
        roll_secant_midpoint_error_rad_s2:midpointError,
        note:'Nominal 20 ms endpoint envelope; frozen inflow/voltage, local roll map. Not instantaneous achievement or a robust reachable-set guarantee.'};
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
    if(config.scenario==='schedule'){
      let command=config.commands[0];
      for(const next of config.commands){if(next.t>t+1e-10)break;command=next;}
      return [command.speed,0,0,command.altitude];
    }
    if(config.scenario==='ramp'){
      const t0=config.ramp_t0??1, Tr=config.ramp_duration_s??2;
      const vx=config.speed*smoothstep((t-t0)/Tr);
      return [vx,0,0,config.altitude??20];
    }
    const vx=config.scenario==='hover'?0:(t<1?0:config.speed);
    return [vx,0,0,config.altitude??20];
  }
  function referenceHorizon(t,config) {
    return Array.from({length:21},(_,k)=>reference(config.preview===false?t:t+k*.05,config));
  }
  function validateOptions(options={}) {
    const cfg={controller:'hybrid',feedback:'truth',scenario:'step',seconds:4,speed:3,altitude:20,
      wind_speed:0,wind_angle:90,wind_vertical_mps:0,seed:42,preview:true,hybrid_actuator_feedback:false,log_hz:50,scales:[1,1,1,1,1,1],
      indi_rpm_desync_ms:0,
      // 표7 센서·구동기 조건 스윕. null이면 번들 프로파일 기본값을 그대로 쓴다
      // (기존 동작과 완전히 동일). gps_dropout_prob·outage는 truth/eskf 둘 다
      // 영향, rpm_feedback_rate_hz는 INDI가 보는 회전수 갱신률(기본 1kHz).
      gps_position_std_m:null, gps_velocity_std_mps:null, gps_rate_hz:null, gps_delay_ms:null,
      gps_dropout_prob:0, gps_outage_start_s:null, gps_outage_duration_s:0,
      rpm_feedback_rate_hz:1000, initial_soc:1.0,
      ...options};
    if(!['hybrid','nmpc','pd','cpid','gslqr','f13','gindi'].includes(cfg.controller))throw new Error('Unknown controller');
    if(!['truth','eskf'].includes(cfg.feedback))throw new Error('Unknown feedback');
    if(!['hover','step','gust','schedule','ramp'].includes(cfg.scenario))throw new Error('Unknown scenario');
    if(!Number.isFinite(cfg.seconds)||cfg.seconds<.001||cfg.seconds>120)throw new Error('Duration must be 0.001–120 seconds');
    if(cfg.scenario==='step'&&cfg.seconds<1.1)throw new Error('속도 계단 시험은 1초 입력 이후를 포함하도록 1.1초 이상 필요합니다.');
    if(cfg.scenario==='gust'&&cfg.seconds<3.1)throw new Error('외란 시험은 2–3초 외란과 종료 이후를 포함하도록 3.1초 이상 필요합니다.');
    if(cfg.scenario==='ramp'){
      const t0=cfg.ramp_t0??1, Tr=cfg.ramp_duration_s??2;
      if(!Number.isFinite(t0)||t0<0)throw new Error('ramp_t0 must be >= 0');
      if(!Number.isFinite(Tr)||Tr<=0)throw new Error('ramp_duration_s must be > 0');
      if(cfg.seconds<t0+Tr+.1)throw new Error('연속 참조 시험은 램프 시작+지속시간 이후를 포함하도록 충분히 길어야 합니다.');
    }
    if(!Number.isFinite(cfg.speed)||cfg.speed<0||cfg.speed>100)throw new Error('Reference speed must be 0–100 m/s');
    if(!Number.isFinite(cfg.altitude)||cfg.altitude<1||cfg.altitude>1000)throw new Error('고도는 1–1000 m 범위입니다.');
    validateWind(cfg);
    // 표8 Q06/Q07 수직풍. environment()의 z성분은 model.py쪽 심볼("environment
    // world xyz")엔 이미 있었지만 JS가 항상 0으로 채웠다 -- 물리 그래프 변경
    // 없이 여기만 고치면 된다.
    if(!Number.isFinite(cfg.wind_vertical_mps)||cfg.wind_vertical_mps<-10||cfg.wind_vertical_mps>10)
      throw new Error('wind_vertical_mps must be -10 to 10');
    // 6번째 요소(모터 속도루프 시간 배율, 표7)는 m/Ixx/Iyy/Izz/CT-CP와 성격이
    // 달라(고의적 불일치가 아니라 실제 있을 수 있는 하드웨어 조건) 범위를 더
    // 넓게 둔다 -- 표7이 5~80ms(공칭 20ms 기준 0.25~4배)까지 요구한다.
    if(!Array.isArray(cfg.scales)||cfg.scales.length!==6||cfg.scales.slice(0,5).some(v=>!Number.isFinite(v)||v<.5||v>1.5)
       ||!Number.isFinite(cfg.scales[5])||cfg.scales[5]<.1||cfg.scales[5]>5)
      throw new Error('Invalid model-error scales');
    if(!Number.isInteger(cfg.seed)||cfg.seed<0||cfg.seed>4294967295)throw new Error('Seed must be an unsigned 32-bit integer');
    if(typeof cfg.preview!=='boolean')throw new Error('Preview must be true or false');
    if(typeof cfg.hybrid_actuator_feedback!=='boolean')throw new Error('Hybrid actuator feedback must be true or false');
    // 표6 S0<->S1: INDI가 쓰는 로터회전수 피드백을 각가속도(자이로) 피드백과
    // 의도적으로 시간축에서 어긋내는 진단 옵션. 0이면 S1(정렬, 기존 기본값과
    // 동일). 잡음 크기·필터 주파수는 이 옵션과 무관하게 고정돼 있다.
    if(!Number.isFinite(cfg.indi_rpm_desync_ms)||cfg.indi_rpm_desync_ms<0||cfg.indi_rpm_desync_ms>200)
      throw new Error('INDI rpm desync must be 0-200 ms');
    // 표7 스윕 인자 검증. null은 "번들 기본값 사용"이라 여기서 범위를 안 본다.
    const optionalPositive=(v,name,max)=>{if(v!==null&&(!Number.isFinite(v)||v<=0||v>max))throw new Error(`${name} must be null or 0-${max}`);};
    optionalPositive(cfg.gps_position_std_m,'gps_position_std_m',10);
    optionalPositive(cfg.gps_velocity_std_mps,'gps_velocity_std_mps',5);
    optionalPositive(cfg.gps_rate_hz,'gps_rate_hz',1000);
    if(cfg.gps_delay_ms!==null&&(!Number.isFinite(cfg.gps_delay_ms)||cfg.gps_delay_ms<0||cfg.gps_delay_ms>2000))
      throw new Error('gps_delay_ms must be null or 0-2000');
    if(!Number.isFinite(cfg.gps_dropout_prob)||cfg.gps_dropout_prob<0||cfg.gps_dropout_prob>1)
      throw new Error('gps_dropout_prob must be 0-1');
    if(cfg.gps_outage_start_s!==null&&(!Number.isFinite(cfg.gps_outage_start_s)||cfg.gps_outage_start_s<0||cfg.gps_outage_start_s>cfg.seconds))
      throw new Error('gps_outage_start_s must be null or within [0, seconds]');
    if(!Number.isFinite(cfg.gps_outage_duration_s)||cfg.gps_outage_duration_s<0||cfg.gps_outage_duration_s>cfg.seconds)
      throw new Error('gps_outage_duration_s must be 0-seconds');
    if(![100,250,500,1000].includes(cfg.rpm_feedback_rate_hz))
      throw new Error('rpm_feedback_rate_hz must be one of 100,250,500,1000');
    if(!Number.isFinite(cfg.initial_soc)||cfg.initial_soc<=0||cfg.initial_soc>1)
      throw new Error('initial_soc must be in (0,1]');
    if(![50,1000].includes(cfg.log_hz)||cfg.log_hz===1000&&cfg.seconds>20)throw new Error('로그는 50 Hz 또는 1 kHz입니다. 상세 1 kHz 로그는 20초 이하로 제한합니다.');
    if(cfg.scenario==='schedule'){
      if(!Array.isArray(cfg.commands)||!cfg.commands.length||cfg.commands.length>6001||cfg.commands[0].t!==0)
        throw new Error('목표 일정은 0초 명령으로 시작해야 합니다.');
      let previous=-1;
      cfg.commands=cfg.commands.map(command=>{
        const {t,speed,altitude}=command||{};
        if(!Number.isFinite(t)||t<=previous||t>cfg.seconds||Math.abs(t/PERIOD/DT-Math.round(t/PERIOD/DT))>1e-7||
           !Number.isFinite(speed)||speed<0||speed>100||!Number.isFinite(altitude)||altitude<1||altitude>1000)
          throw new Error('목표 일정의 시각·속도·고도를 확인하세요. 시각은 증가하는 20 ms 격자여야 합니다.');
        // Older logs have no wind fields: their documented environment was calm.
        const wind_speed=command.wind_speed??cfg.wind_speed,wind_angle=command.wind_angle??cfg.wind_angle;
        validateWind({wind_speed,wind_angle});
        previous=t;return {t,speed,altitude,wind_speed,wind_angle};
      });
    }else delete cfg.commands;
    return cfg;
  }
  function validateWind({wind_speed,wind_angle}){
    if(!Number.isFinite(wind_speed)||wind_speed<0||wind_speed>30||
       !Number.isFinite(wind_angle)||wind_angle<0||wind_angle>360)
      throw new Error('풍속은 0–30 m/s, 바람 방향은 0–360° 범위입니다.');
  }
  function environment(t, config) {
    let command=config;
    if(config.scenario==='schedule')for(const c of config.commands){if(c.t>t+1e-10)break;command=c;}
    const speed=command.wind_speed??config.wind_speed??0,angle=(command.wind_angle??config.wind_angle??90)*Math.PI/180;
    const vertical=command.wind_vertical_mps??config.wind_vertical_mps??0;
    // Direction TO which air flows, in world axes; not meteorological FROM.
    // env[2]>0 means air moving up (world +z), matching the world +z-up convention.
    const env=[speed*Math.cos(angle),speed*Math.sin(angle),vertical,0,0,0];
    if(config.scenario==='gust'&&t>=2&&t<3){env[1]+=3;env[3]=.015;env[4]=.025;}
    return env;
  }
  function percentile(values, q) {
    if(!values.length) return null;
    const sorted=values.slice().sort((a,b)=>a-b);
    return sorted[Math.floor((sorted.length-1)*q)];
  }

  // Pacing controls ONLY when the next fixed-size block may start. A suspended
  // tab or slow machine never creates a large dt or a catch-up flight jump.
  class ObservationClock {
    constructor(now=()=>performance.now(),sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms))){
      this.now=now;this.sleep=sleep;this.reset(0);
    }
    reset(t){this.lastTime=t;this.lastWall=this.now();}
    async wait(t,interrupted=()=>false){
      const deadline=this.lastWall+Math.max(0,t-this.lastTime)*1000;
      while(!interrupted()&&this.now()<deadline)await this.sleep(Math.min(20,deadline-this.now()));
      this.reset(t);
    }
  }

  async function run(ca,data,options={},progress=()=>{},shouldStop=()=>false,hooks={}) {
    const cfg=validateOptions(options);
    const b=makeBindings(ca,data,['pd','cpid','gslqr','gindi'].includes(cfg.controller)?[]:[cfg.controller]);
    try {
    const optimizer=cfg.controller==='pd'?new ObservationController(b,data)
      :cfg.controller==='cpid'?new CPID(b,data)
      :cfg.controller==='gslqr'?new GSLQR(b,data)
      :cfg.controller==='gindi'?new GINDI(b,data)
      :cfg.controller==='f13'?new F13Adapter(ca,b,data)
      :new Optimizer(ca,b,data,cfg.controller);
    const indi=['nmpc','cpid','gslqr'].includes(cfg.controller)?null:new INDI(b,data);
    const initial=data.initial.slice();initial[2]=cfg.altitude;initial[17]=cfg.initial_soc;
    const p=data.profile;
    // 표7: null이면 번들 프로파일 기본값 그대로(기존 동작과 동일). eskf 피드백의
    // ESKF 자체 R(측정공분산, research/eskf.py)은 번들에 고정돼 있어 이 스윕과
    // 별개다 -- 추정기의 잡음 가정이 아니라 "실제로 얼마나 나쁜 측정을 받는지"만
    // 바꾼다는 뜻. gps_dropout_prob/outage는 research/ACTUATOR_FEEDBACK.md류
    // 기존 기능이 아니라 이번에 새로 추가한 것.
    const s={...data.sensors,
      gps_position_std_m: cfg.gps_position_std_m ?? data.sensors.gps_position_std_m,
      gps_velocity_std_mps: cfg.gps_velocity_std_mps ?? data.sensors.gps_velocity_std_mps,
      gps_period_ticks: cfg.gps_rate_hz ? Math.max(1,Math.round(1/(cfg.gps_rate_hz*DT))) : data.sensors.gps_period_ticks,
      gps_latency_ticks: cfg.gps_delay_ms!==null ? Math.round(cfg.gps_delay_ms/1000/DT) : data.sensors.gps_latency_ticks};
    // ESKF는 GPS 표본의 취득 시각(capture tick)까지 거슬러 재적분해야 하므로,
    // 지연(gps_latency_ticks)보다 짧게 이력을 지우면 표7의 큰 지연 조건에서
    // "GPS timestamp is outside the ESKF history"로 죽는다 -- 직접 겪은 오류.
    const estimator=new Estimator(b,data,initial,Math.max(200,s.gps_latency_ticks+50));
    const gpsDropoutNoise=uniformRandom(cfg.seed+5);
    const gpsOutageEndTick=cfg.gps_outage_start_s!==null?Math.round((cfg.gps_outage_start_s+cfg.gps_outage_duration_s)/DT):-1;
    const gpsOutageStartTick=cfg.gps_outage_start_s!==null?Math.round(cfg.gps_outage_start_s/DT):-1;
    const rpmUpdatePeriodTicks=Math.max(1,Math.round(1000/cfg.rpm_feedback_rate_hz));
    const imuNoise=random(cfg.seed), gpsNoise=random(cfg.seed+1), rpmNoise=random(cfg.seed+2), biasNoise=random(cfg.seed+3);
    // Independent gyro stream: both controllers see the same noise realization,
    // including their observation at t=0, without borrowing a previous-tick gyro.
    const gyroNoise=random(cfg.seed+4);
    let ba=Array.from({length:3},()=>s.accel_bias_std_mps2*biasNoise());
    let bg=Array.from({length:3},()=>s.gyro_bias_std_rad_s*biasNoise());
    let x=initial.slice(), sensedGyro, sensedRpm;
    // 표6 S0<->S1: rpm 피드백만 desyncTicks만큼 지연시켜 INDI에 전달한다
    // (자이로/각가속도 쪽은 그대로) -- "정렬된 입력·각가속도"와 "의도한 시간
    // 불일치"를 가르는 유일한 차이. desyncTicks=0(S1)이면 버퍼를 아예 안 써서
    // 기존 동작과 바이트 단위로 동일하다.
    const desyncTicks=Math.round(cfg.indi_rpm_desync_ms/1000/DT);
    const rpmHistory=[];
    function desyncedRpmFeedback(feedback){
      rpmHistory.push(feedback.slice(13,17));
      if(rpmHistory.length>desyncTicks+1)rpmHistory.shift();
      if(desyncTicks<=0||rpmHistory.length<=desyncTicks)return feedback;
      return [...feedback.slice(0,13),...rpmHistory[0]];
    }
    // 표7 회전수 피드백 갱신률: 기본 1kHz(매 틱)를 낮추면 sensedRpm이 다음
    // 갱신 틱까지 held(직전 값 유지)된다. 자이로는 이 스윕과 무관 -- 그래야
    // "회전수 피드백 갱신률"만의 효과를 본다.
    let rpmCaptureCount=0;
    function captureRates(){
      sensedGyro=x.slice(10,13).map((v,i)=>v+bg[i]+s.gyro_std_rad_s*gyroNoise());
      if(rpmCaptureCount%rpmUpdatePeriodTicks===0)
        sensedRpm=x.slice(13,17).map(v=>v+s.rpm_std_rad_s*rpmNoise());
      rpmCaptureCount++;
    }
    captureRates();
    if(indi) indi.gyro=cfg.feedback==='eskf'?sensedGyro.slice():x.slice(10,13);
    let u=x.slice(13,17), request=optimizer.previous.slice(), voltage=p.battery.series*4.2;
    let samples=0, sumV2=0, sumZ2=0, sumEstimate2=0, energy=0, outside=0, maxEnergyResidual=0, maxBusResidual=0;
    let failure=null, stopped=false, saturation=0, imuSamples=0, gpsSamples=0, maxRate=0,outerUpdates=0;
    let currentLimited=0,voltageLimited=0,trackingLimited=0,coasting=0,allocationLimited=0,allocationThrustResidual=0,allocationAlphaResidual=0;
    let thrustTrackingSquared=0,alphaTrackingSquared=0;
    const pendingGPS=[], gpsEvents=[], trace=[], ticks=Math.round(cfg.seconds/DT), begin=performance.now();
    const yieldMessages=()=>new Promise(resolve=>setTimeout(resolve,0));
    function record(t,d,dx,imu=null){
      // All channels in a row refer to x(t), not a mixture of x(t) and x(t+dt).
      trace.push({t,state:x.slice(),position_m:x.slice(0,3),v:x.slice(3,6),z:x[2],q_xyzw:x.slice(6,10),
        body_rate_rad_s:x.slice(10,13),rotor_rad_s:x.slice(13,17),command_rad_s:u.slice(),
        reference:reference(t,cfg),environment:environment(t,cfg),virtual:indi?request.slice():null,soc:x[17],
        motor:motorDiagnostics(d),alpha_rad_s2:dx.slice(10,13),
        estimate:estimator.state.slice(),covariance_diagonal:Array.from({length:15},(_,i)=>estimator.cov[i*16]),
        sensor:{capture_t_s:t,gyro_t_s:t,rotor_t_s:t,imu_t_s:imu?t:null,
          gyro_rad_s:sensedGyro.slice(),rotor_rad_s:sensedRpm.slice(),imu,
          true_accel_bias_mps2:ba.slice(),true_gyro_bias_rad_s:bg.slice()},
        allocation:indi?.last||null});
    }
    function report(t){progress({t,total:cfg.seconds,v:x.slice(3,6),z:x[2],p:x.slice(0,3),q:x.slice(6,10),rpm:x.slice(13,17),solves:optimizer.stats.length,
      reference:reference(t,cfg),environment:environment(t,cfg),soc:x[17],voltage,energy_J:energy,outside_map_fraction:samples?outside/samples:0,
      current_limited_fraction:samples?currentLimited/samples:0,wall_seconds:(performance.now()-begin)/1000});}
    for(let tick=0;tick<ticks;tick++) {
      const t=tick*DT;
      // Controller receives this explicit observation. No true-state closure in it.
      const feedback=cfg.feedback==='truth'?x.slice(0,17):estimator.feedback(sensedGyro,sensedRpm);
      if(tick%PERIOD===0) {
        await yieldMessages();
        if(hooks.beforeControl)await hooks.beforeControl(t);
        if(shouldStop()){stopped=true;break;}
        const command=hooks.command?.();
        if(command){
          if(cfg.scenario!=='schedule'||cfg.preview)throw new Error('Live commands require a non-preview schedule');
          const previousReference=reference(t,cfg);
          const previousCommand=cfg.commands.filter(c=>c.t<=t).at(-1);
          const applied={...previousCommand,...command,t};
          const commands=cfg.commands.filter(c=>c.t<t);
          commands.push(applied);
          cfg.commands=validateOptions({...cfg,commands}).commands;
          // The preceding tick already sampled velocity error at x(t). Update
          // that one boundary sample when a newly received command starts at t,
          // matching a replay that knew the timestamped schedule in advance.
          if(samples)sumV2+=sub(x.slice(3,6),reference(t,cfg).slice(0,3)).reduce((s,v)=>s+v*v,0)
            -sub(x.slice(3,6),previousReference.slice(0,3)).reduce((s,v)=>s+v*v,0);
          hooks.commandApplied?.({...cfg.commands.at(-1)});
        }
        // Delta-input cost starts from the last achieved nominal allocation,
        // not an impossible requested virtual action. This is not plant truth.
        if(indi?.last)optimizer.previous=indi.last.allocated.slice();
        const envelope=cfg.controller==='hybrid'&&cfg.hybrid_actuator_feedback?indi.envelope(feedback,voltage):null;
        if(envelope)envelope.t_s=t;
        if(globalThis.__DEBUG_SOLVE__)console.error(`[solve] t=${t.toFixed(3)} feedback=${JSON.stringify(feedback)}`);
        const __t0=Date.now();
        request=optimizer.solve(feedback,referenceHorizon(t,cfg),voltage,[0,0,0],envelope);
        if(globalThis.__DEBUG_SOLVE__)console.error(`[solve] done in ${Date.now()-__t0}ms status=${optimizer.stats.at(-1)?.status} request=${JSON.stringify(request)}`);
        outerUpdates++;
        const stat=optimizer.stats.at(-1);if(stat)stat.t_s=t;
        report(t);
        await yieldMessages();
        if(shouldStop()){stopped=true;break;}
      }
      // Apply a live wind change on this tick, exactly as its recorded replay.
      const env=environment(t,cfg);
      const target=reference(t,cfg);
      u=indi?indi.update(desyncedRpmFeedback(feedback),request,cfg.feedback==='eskf'?sensedGyro:feedback.slice(10,13)):request.slice();
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
        // Evaluation may use plant truth; the controller/envelope never receives
        // these accelerations. Measure lag separately from static allocation.
        thrustTrackingSquared+=(request[0]-motor.thrust_N.reduce((s,v)=>s+v,0))**2;
        alphaTrackingSquared+=sub(request.slice(1),dx.slice(10,13)).reduce((s,v)=>s+v*v,0);
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
        // 표7 GNSS 누락: 매 표본 베르누이 확률(gps_dropout_prob) 또는 예약된
        // 연속 구간(gps_outage_*, "1초 연속" 조건) 중 하나라도 걸리면 이번
        // 표본을 통째로 버린다 -- pendingGPS에도 안 들어가 추정기가 아예 못 본다.
        const scheduledOutage=gpsOutageStartTick>=0&&nextTick>=gpsOutageStartTick&&nextTick<gpsOutageEndTick;
        const dropped=scheduledOutage||(cfg.gps_dropout_prob>0&&gpsDropoutNoise()<cfg.gps_dropout_prob);
        if(!dropped){
          pendingGPS.push({capture:nextTick,arrival:nextTick+s.gps_latency_ticks,
            value:x.slice(0,6).map((v,i)=>v+(i<3?s.gps_position_std_m:s.gps_velocity_std_mps)*gpsNoise())});
        }
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
      implementation:data.provenance||null,solver:cfg.controller==='pd'?'PD–INDI observation controller (not NMPC)':'CasADi/IPOPT WebAssembly 3.8.0',
      simulated_seconds:samples*DT,wall_seconds:(performance.now()-begin)/1000,failure,
      status:failure?'failed':stopped?'stopped':'completed',
      event_coverage:{speed_step:['step','gust'].includes(cfg.scenario)&&end>1,
        speed_ramp_completed:cfg.scenario==='ramp'&&end>=(cfg.ramp_t0??1)+(cfg.ramp_duration_s??2),
        gust_started:cfg.scenario==='gust'&&end>2,gust_completed:cfg.scenario==='gust'&&end>=3},
      counts:{plant:samples,imu:imuSamples,nmpc:optimizer.stats.length,outer:outerUpdates,indi:indi?.samples||0,gps_captured:gpsSamples,
              gps_fused:estimator.updates,gps_replayed_imu:estimator.replayed},
      metrics:{velocity_rmse_mps:rmse(sumV2),altitude_rmse_m:rmse(sumZ2),
               position_estimation_rmse_m:rmse(sumEstimate2),electrical_energy_J:energy,
               saturation_fraction:fraction(saturation),outside_prop_map_fraction:fraction(outside),max_body_rate_rad_s:maxRate,
               current_limited_fraction:fraction(currentLimited),voltage_limited_fraction:fraction(voltageLimited),
               tracking_limited_fraction:fraction(trackingLimited),coasting_fraction:fraction(coasting),
               allocation_limited_fraction:indi?fraction(allocationLimited):null,
               allocation_thrust_residual_max_N:indi?allocationThrustResidual:null,
               allocation_alpha_residual_max_rad_s2:indi?allocationAlphaResidual:null,
               virtual_thrust_tracking_rmse_N:indi?rmse(thrustTrackingSquared):null,
               virtual_alpha_tracking_rmse_rad_s2:indi?rmse(alphaTrackingSquared):null,
               actuator_envelope_unavailable:cfg.controller==='hybrid'&&cfg.hybrid_actuator_feedback?
                 optimizer.stats.filter(s=>s.actuator_envelope&&!s.actuator_envelope.available).length:0,
               motor_energy_residual_W:maxEnergyResidual,bus_residual_V:maxBusResidual,
               solver_failures:optimizer.stats.filter(s=>!s.success).length,
               solve_ms_p50:percentile(timing,.5),solve_ms_p95:percentile(timing,.95),solve_ms_max:timing.length?Math.max(...timing):null,
               deadline_misses_20ms:timing.filter(t=>t>20).length},
      final:x,trace,solves:optimizer.stats,gps_events:gpsEvents,
      units:{state:'p[m] v[m/s] q[xyzw] omega[rad/s] rotor[rad/s] SOC[1]',imu:'specific force[m/s²], gyro[rad/s]',
        covariance_diagonal:'15D error: p, v, attitude(rad), accel bias, gyro bias',virtual:'T[N], angular acceleration[rad/s²]'},
      limitations:[...p.assumptions,...(cfg.controller==='pd'?['PD–INDI is a nominal observation controller, not NMPC–INDI Hybrid. Its gains are illustrative, not experimentally identified.']:[]),
                    'Simulation time is paused while IPOPT solves; wall deadline statistics are NOT a hard-real-time guarantee.',
                    'Allocation reports nominal static rotor forces plus measured angular-acceleration increments, NOT instant actuator achievement. Motor spool reaction is not explicitly allocated.',
                    'Version 2 logs use synchronized gyro/RPM timestamps and independent sensor noise streams; old and new seed traces are not identical.',
                    'GPS innovation and NIS are logged but outliers are not rejected. Contact is only a CG ground-crossing abort, not STL collision dynamics.',
                    '17-state NMPC holds measured bus voltage over its 1 s horizon. Wind is unobserved and assumed zero by both controllers.',
                    'Hybrid actuator feedback constrains only the first virtual input using a nominal 20 ms endpoint motor forecast. Voltage/inflow are frozen; roll torque is locally approximated. It neither predicts motor lag through the full horizon nor guarantees instantaneous/robust feasibility. Unavailable envelopes and held-command violations are logged.',
                    'Known initial pose/alignment; IMU Euler propagation, timestamped GPS correction and replay; GPS delay is 20 ms.',
                    'Outside-map samples use an explicit passive continuation; these are NOT validated propulsion performance results.']};
    } finally {b.dispose();}
  }
  const api={run,makeBindings,boundedIncrement,allocateThrust,motorDiagnostics,validateOptions,referenceHorizon,Estimator,Optimizer,ObservationController,ObservationClock,INDI,rotate,random,reference,environment,shiftPrediction,smoothstep};
  if(typeof module!=='undefined'&&module.exports) module.exports=api;
  else root.ResearchRuntime=api;
})(globalThis);
