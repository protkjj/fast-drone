"""Existing direct/virtual NMPC formulations with explicit v2 experiment settings.

Virtual commands remain [T, angular acceleration]. Four auxiliary rotor rates
per stage certify nonlinear wrench feasibility; they are NOT motor commands
sent to the plant. The inner INDI alone generates the actual motor commands.
No motor-lag reachability guarantee in the 13-state virtual predictor.
"""
from time import perf_counter
import casadi as ca
import numpy as np

from models.team_light.control.dynamics import build_dynamics, _quat_to_rotmat, _body_aerodynamics, _rotor_forces_moments
from models.team_light.control.hybrid_comparison import build_virtual_dynamics
from models.team_light.control.propeller_curve import positive_thrust_j_limit, uses_curve


def virtual_wrench_residual(x13, virtual, auxiliary_rates, p):
    vb = _quat_to_rotmat(x13[6:10]).T@x13[3:6]
    omega = x13[10:13]
    _, ma = _body_aerodynamics(vb, omega, p)
    fr, mr = _rotor_forces_moments(vb, auxiliary_rates, omega, p)
    inertia = ca.DM([p['Ixx'],p['Iyy'],p['Izz']])
    return ca.vertcat(fr[0]-virtual[0],
                      ma+mr-ca.cross(omega,inertia*omega)-inertia*virtual[1:4])


def positive_thrust_domain(x, rates, p):
    vb = _quat_to_rotmat(x[6:10]).T@x[3:6]
    return positive_thrust_j_limit(p)*rates*p['D_prop']/(2*ca.pi)-ca.fmax(vb[0],0.)


class ComparisonNMPC:
    """CasADi/IPOPT multiple shooting; common horizon/state cost for both forms."""
    def __init__(self, p, trim, *, virtual=False, z_ref=20., N=15,
                 dt_prediction=.04, dt_control=.04, max_iter=80):
        if not uses_curve(p):
            raise ValueError('This experiment adapter is for the v2 curve model only.')
        self.p, self.virtual, self.N = p, virtual, N
        self.dt_ctrl, self.dt_prediction = dt_control, dt_prediction
        self.nx = 13 if virtual else 17
        self.stride = self.nx+4+(4 if virtual else 0)
        self.v_ref, self.z_ref = trim['state'][3:6].copy(), float(z_ref)
        self.n_ref, self.x_ref = trim['control'].copy(), trim['state'].copy()
        self.x_ref[2] = z_ref
        self.u_ref = np.r_[trim['T_total'],0.,0.,0.] if virtual else self.n_ref.copy()
        self.input_scale = np.r_[p['mass']*p['g'],[20.]*3] if virtual else np.full(4,p['n_max'])
        self.state_scale = np.r_[[100.]*3,[85.]*3,[1.]*4,[20.]*3, [] if virtual else [p['n_max']]*4]
        self.wrench_scale = np.r_[p['mass']*p['g'],20*np.array([p['Ixx'],p['Iyy'],p['Izz']])]
        self._build(max_iter)
        self.reset()

    def _build(self,max_iter):
        p, nx, N = self.p,self.nx,self.N
        f, _, _ = build_virtual_dynamics(p) if self.virtual else build_dynamics(p)
        xx, uu = ca.SX.sym('xx',nx), ca.SX.sym('uu',4)
        xn = xx
        # Same prediction integration interval; full plant motor lag is retained.
        h = self.dt_prediction/2
        for _ in range(2):
            k1=f(xn,uu); k2=f(xn+h*k1/2,uu); k3=f(xn+h*k2/2,uu); k4=f(xn+h*k3,uu)
            xn=xn+h*(k1+2*k2+2*k3+k4)/6
        self.F=ca.Function('comparison_step',[xx,uu],[xn])
        param=ca.SX.sym('reference',nx+3+1+4)
        x0,vr,zr,ur=param[:nx],param[nx:nx+3],param[nx+3],param[nx+4:]
        w,lb,ub,g,lg,ug=[],[],[],[],[],[]
        cost=0.; xp=x0; up=ur
        if self.virtual:
            limits=np.r_[4*p['k_T']*p['n_max']**2, [100.]*3]
            input_min=np.r_[0.,[-100.]*3]/self.input_scale
            input_max=limits/self.input_scale
            r=np.r_[1/(.3*p['mass']*p['g'])**2,[1/20.**2]*3]
        else:
            input_min=np.full(4,p['n_min']/p['n_max'])
            input_max=np.ones(4)
            r=np.full(4,1/(.15*p['n_max'])**2)
        for k in range(N):
            y=ca.SX.sym(f'command_{k}',4)
            w.append(y); lb.extend(input_min); ub.extend(input_max)
            u=y*ca.DM(self.input_scale)
            if self.virtual:
                a=ca.SX.sym(f'feasible_rotors_{k}',4)
                w.append(a); lb.extend([p['n_min']/p['n_max']]*4); ub.extend([1.]*4)
                rates=a*p['n_max']
                residual=virtual_wrench_residual(xp,u,rates,p)/ca.DM(self.wrench_scale)
                g.append(residual); lg.extend([0.]*4); ug.extend([0.]*4)
                # Same domain restriction as full NMPC; no windmill torque credit.
                g.append(positive_thrust_domain(xp,rates,p)/85.)
                lg.extend([0.]*4); ug.extend([np.inf]*4)
            x=ca.SX.sym(f'node_{k+1}',nx)
            w.append(x); lb.extend([-1e6]*nx); ub.extend([1e6]*nx)
            if not self.virtual:
                lb[-4:]=[p['n_min']]*4; ub[-4:]=[p['n_max']]*4
                g.append(positive_thrust_domain(x,x[13:17],p)/85.)
                lg.extend([0.]*4); ug.extend([np.inf]*4)
            g.append((x-self.F(xp,u))/ca.DM(self.state_scale))
            lg.extend([0.]*nx); ug.extend([0.]*nx)
            dv=x[3:6]-vr; dz=x[2]-zr
            cost+=ca.dot(dv*ca.DM([5.,5.,10.]),dv)+20*dz**2+ca.dot(x[10:13],x[10:13])
            cost+=ca.dot((u-ur)*ca.DM(r),u-ur)+.1*ca.dot((u-up)*ca.DM(r),u-up)
            xp,up=x,u
        dv=xp[3:6]-vr
        cost+=10*(ca.dot(dv*ca.DM([5.,5.,10.]),dv)+20*(xp[2]-zr)**2)
        decision,con=ca.vertcat(*w),ca.vertcat(*g)
        self.solver=ca.nlpsol('comparison_virtual' if self.virtual else 'comparison_direct','ipopt',
                             {'x':decision,'p':param,'g':con,'f':cost},
                             {'ipopt.print_level':0,'ipopt.sb':'yes','print_time':False,
                              'ipopt.max_iter':max_iter,'ipopt.tol':1e-6,
                              'ipopt.acceptable_tol':1e-5,'ipopt.warm_start_init_point':'yes'})
        self.constraints=ca.Function('feasibility_check',[decision,param],[con])
        self.lbw,self.ubw,self.lbg,self.ubg=map(np.asarray,(lb,ub,lg,ug))

    def set_reference(self, v_ref, z_ref, x_trim, u_trim, thrust):
        self.v_ref=np.array(v_ref); self.z_ref=float(z_ref)
        self.x_ref=x_trim.copy(); self.x_ref[2]=z_ref; self.n_ref=u_trim.copy()
        self.u_ref=np.r_[thrust,0.,0.,0.] if self.virtual else self.n_ref.copy()

    def _seed(self):
        nodes=[]
        for k in range(self.N):
            x=self.x_ref[:self.nx].copy()
            x[:3]+=self.v_ref*(k+1)*self.dt_prediction
            nodes.extend((self.u_ref/self.input_scale).tolist())
            if self.virtual:
                nodes.extend((self.n_ref/self.p['n_max']).tolist())
            nodes.extend(x.tolist())
        return np.asarray(nodes)

    def reset(self):
        self.w0=self._seed(); self._last_t=-np.inf; self._u_current=self.u_ref.copy()
        self.solve_log=[]; self.consec_fail=0; self.last_status='none'; self.last_solution=None

    def __call__(self,t,x):
        if t-self._last_t >= self.dt_ctrl-1e-8:
            self._last_t=t
            values=np.r_[x[:self.nx],self.v_ref,self.z_ref,self.u_ref]
            started=perf_counter()
            try:
                sol=self.solver(x0=self.w0,lbx=self.lbw,ubx=self.ubw,lbg=self.lbg,ubg=self.ubg,p=values)
                answer=np.array(sol['x']).reshape(-1)
                constraints=np.array(sol['g']).reshape(-1)
                violation=float(max(np.max(self.lbg-constraints),np.max(constraints-self.ubg),0.))
                status=self.solver.stats().get('return_status','unknown')
                accepted=(status in ('Solve_Succeeded','Solved_To_Acceptable_Level')
                          and np.all(np.isfinite(answer)) and violation < 1e-4)
            except RuntimeError as exc:
                status='exception: '+str(exc)[:100]; accepted=False; violation=None; answer=None
            self.solve_log.append(dict(t=float(t),status=status,accepted=bool(accepted),
                                       seconds=perf_counter()-started,scaled_constraint_violation=violation))
            self.last_status=status
            if accepted:
                self._u_current=answer[:4]*self.input_scale
                self.w0=np.r_[answer[self.stride:],answer[-self.stride:]]
                self.last_solution=answer.copy(); self.consec_fail=0
            else:
                self.consec_fail+=1  # Explicit hold, no hidden LQR rescue.
        return self._u_current.copy()
