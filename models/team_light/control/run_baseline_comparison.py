"""Frozen-aircraft preliminary comparisons. No MC, no implicit legacy model.

python -m models.team_light.control.run_baseline_comparison --stage lqr|pilot|short|all
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy.spatial.transform import Rotation

from models.team_light.control.baseline_v2 import baseline_params, PARAMETER_SHA256, parameter_hash
from models.team_light.control.trim import find_trim
from models.team_light.control.controller import LQRController, ScheduledLQR, INDIController
from models.team_light.control.hybrid_comparison import ProperHybrid, NaiveHybrid
from models.team_light.control.comparison_nmpc import ComparisonNMPC
from models.team_light.control.dynamics import AxialDronePlant
from models.team_light.control.geometry import rotor_thrusts
from models.team_light.control.propeller_curve import domain_status
from models.team_light.control.serialization import jsonable, lqr_weights
from models.team_light.control.mission_sim import MissionProfile

ROOT=Path(__file__).resolve().parents[1]
LABELS=('LQR70','GS-LQR','INDI','NMPC','Naive','Split')
GRID=np.array([0,10,20,30,40,50,60,70,75,80,85], dtype=float)
DT=.002


class Factory:
    def __init__(self,p):
        self.p=p
        self.Q,self.R=lqr_weights(p)
        self.scheduled=ScheduledLQR(p,[70,0,0],20.,GRID,self.Q,self.R)
        self.trim70=find_trim(p,70.)
        self.fixed=LQRController(p,self.trim70['state'],self.trim70['control'],self.Q,self.R)
        self.solvers={}
        self.lqr_gains=[dict(speed=float(v),max_real_eigenvalue=float(np.max(np.real(
            np.linalg.eigvals(LQRController(p,x,u,self.Q,self.R).A_r-
                             LQRController(p,x,u,self.Q,self.R).B_r@k.reshape(4,14))))))
            for v,x,u,k in zip(GRID,self.scheduled._x_trim_arr,self.scheduled._u_trim_arr,self.scheduled._K_r_flat)]

    def ref_trim(self,v,z):
        _,x,n=self.scheduled._interpolate(float(v[0]))
        x[2]=z
        vb=Rotation.from_quat(x[6:10]).as_matrix().T@x[3:6]
        return x,n,float(np.sum(rotor_thrusts(self.p,n,vb)))

    def make(self,label,speed,z):
        tr=find_trim(self.p,speed)
        if label=='LQR70':
            ctrl=deepcopy(self.fixed)
        elif label=='GS-LQR':
            ctrl=deepcopy(self.scheduled)
        elif label=='INDI':
            ctrl=INDIController(self.p,[speed,0,0],z,dt=DT)
            ctrl.max_tilt=np.radians(100.)
        else:
            virtual=label=='Split'
            key='virtual' if virtual else 'direct'
            if key not in self.solvers:
                print('  Building '+key+' NMPC...',flush=True)
                self.solvers[key]=ComparisonNMPC(self.p,tr,virtual=virtual,z_ref=z)
            solver=self.solvers[key]
            solver.set_reference([speed,0,0],z,tr['state'],tr['control'],tr['T_total'])
            solver.reset()
            ctrl=ProperHybrid(solver,self.p,dt=DT) if virtual else (
                NaiveHybrid(solver,self.p,dt=DT) if label=='Naive' else solver)
        self.update(ctrl,[speed,0,0],z)
        return ctrl

    def update(self,ctrl,v,z):
        x,n,thrust=self.ref_trim(v,z)
        if isinstance(ctrl,LQRController):
            ctrl.x_trim=x; ctrl.u_trim=n
        elif isinstance(ctrl,ScheduledLQR):
            ctrl.v_ref=np.asarray(v); ctrl.z_ref=z
        elif isinstance(ctrl,INDIController):
            ctrl.v_ref=np.asarray(v); ctrl.z_ref=z
            ctrl.trim_force_ref=Rotation.from_quat(x[6:10]).as_matrix()[:,0]*thrust
        else:
            solver=ctrl.nmpc if hasattr(ctrl,'nmpc') else ctrl
            solver.set_reference(v,z,x,n,thrust)


def gust(t,axis,peak,start=1.5):
    w=np.zeros(3)
    if start <= t <= start+1.:
        w[axis]=peak*.5*(1-np.cos(2*np.pi*(t-start)))
    return w


def solver_of(ctrl):
    obj=ctrl.nmpc if hasattr(ctrl,'nmpc') else ctrl
    return obj if isinstance(obj,ComparisonNMPC) else None


def run_case(factory,label,speed,case,out,*,mission=False):
    p=factory.p; truth=deepcopy(p)
    if case=='aero_plus': truth['aero_scale']=1.15
    if case=='aero_minus': truth['aero_scale']=.85
    profile=MissionProfile(70.,50.) if mission else None
    duration=profile.T_total if mission else 4.
    initial_speed=0. if mission else speed
    z=2. if mission else 20.
    tr=find_trim(p,initial_speed)
    x=tr['state'].copy(); x[2]=z
    if not mission: x[2]+=.2; x[3]+=.5
    ctrl=factory.make(label,initial_speed,z); solver=solver_of(ctrl)
    plant=AxialDronePlant(truth,dt=DT)
    times=[0.]; states=[x.copy()]; commands=[]; refs=[]; winds=[]
    reason=None; outside=0; start=perf_counter()
    for k in range(round(duration/DT)):
        t=k*DT
        v,zref,_=profile.get_ref(t) if mission else (np.array([speed,0.,0.]),20.,'cruise')
        if mission: factory.update(ctrl,v,zref)
        w=gust(t,2,10.,35.) if mission else (
            gust(t,2,2.) if case=='vertical' else gust(t,1,2.) if case=='lateral' else np.zeros(3))
        try:
            u=np.asarray(ctrl(t,x))
            if not np.all(np.isfinite(u)):
                reason='nonfinite command'; break
            if np.any(u < p['n_min']-1e-6) or np.any(u > p['n_max']+1e-6):
                reason='command outside physical bounds'; break
            if solver is not None and solver.consec_fail >= 5:
                reason='5 consecutive optimizer failures'; break
            xn=plant.step(x,u,w)
        except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
            reason=type(exc).__name__+': '+str(exc)[:160]; break
        if not np.all(np.isfinite(xn)):
            reason='nonfinite state'; break
        vb=Rotation.from_quat(xn[6:10]).as_matrix().T@(xn[3:6]-w)
        outside+=not domain_status(p,xn[13:17],vb)['inside_assumed_domain']
        commands.append(u.copy()); refs.append(np.r_[v,zref]); winds.append(w)
        x=xn; times.append((k+1)*DT); states.append(x.copy())
        if abs(x[2]-zref) > (15. if mission else 5.): reason='altitude deviation limit'
        elif np.linalg.norm(x[3:6]-v) > (35. if mission else 20.): reason='velocity error limit'
        elif np.linalg.norm(x[10:13]) > 30.: reason='body rate limit'
        if reason: break
    times,states=np.asarray(times),np.asarray(states)
    commands=np.asarray(commands).reshape(-1,4)
    refs=np.asarray(refs).reshape(-1,4)
    # Align reference with STATE time, not the previous control sample.
    if mission:
        vr,zr=profile.compute_refs(times)
    else:
        vr=np.tile([speed,0,0],(len(times),1)); zr=np.full(len(times),20.)
    ez=states[:,2]-zr; ev=states[:,3:6]-vr
    saturation=float(np.mean(np.any((commands <= p['n_min']+1e-6) |
                                   (commands >= p['n_max']-1e-6),axis=1))) if len(commands) else 0.
    log=deepcopy(solver.solve_log) if solver is not None else []
    result=dict(controller=label,speed_m_s=speed,case=case,mission=mission,
                completed=reason is None,stop_reason=reason,elapsed_sim_s=float(times[-1]),
                wall_seconds=perf_counter()-start,z_rmse_m=float(np.sqrt(np.mean(ez**2))),
                z_max_m=float(np.max(abs(ez))),vx_rmse_m_s=float(np.sqrt(np.mean(ev[:,0]**2))),
                velocity_rmse_m_s=float(np.sqrt(np.mean(np.sum(ev**2,axis=1)))),
                final_velocity_error_m_s=float(np.linalg.norm(ev[-1])),final_z_error_m=float(ez[-1]),
                saturation_fraction=saturation,prop_domain_outside_fraction=outside/max(len(commands),1),
                optimizer_failures=sum(not a['accepted'] for a in log),optimizer_calls=len(log),
                solve_log=log,parameter_sha256=parameter_hash(p))
    result['tracking_pass']=bool(reason is None and abs(ez[-1]) < .5 and
                                 np.linalg.norm(ev[-1]) < .5 and saturation < .01)
    tag=f'{label}_{speed:g}_{case}'+('_mission' if mission else '')
    np.savez_compressed(out/(tag+'.npz'),t=times,x=states,u=commands,v_ref=vr,z_ref=zr,
                        wind=np.asarray(winds).reshape(-1,3))
    (out/(tag+'.json')).write_text(json.dumps(jsonable(result),ensure_ascii=False,indent=2),encoding='utf-8')
    print(f'  {tag}: completed={result["completed"]}, z_RMSE={result["z_rmse_m"]:.4f}, '
          f'v_RMSE={result["velocity_rmse_m_s"]:.3f}, sat={100*saturation:.2f}%, '
          f'solver_fail={result["optimizer_failures"]}/{len(log)}, stop={reason}',flush=True)
    return result


def plots(out,results):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    groups=sorted({(r['speed_m_s'],r['case'],r['mission']) for r in results})
    for speed,case,mission in groups:
        fig,ax=plt.subplots(3,1,figsize=(9,8),sharex=True)
        for r in results:
            if (r['speed_m_s'],r['case'],r['mission']) != (speed,case,mission): continue
            label=r['controller']; tag=f'{label}_{speed:g}_{case}'+('_mission' if mission else '')
            data=np.load(out/(tag+'.npz'))
            legend=label+(' [stopped]' if not r['completed'] else '')
            ax[0].plot(data['t'],data['x'][:,2]-data['z_ref'],label=legend)
            ax[1].plot(data['t'],data['x'][:,3]-data['v_ref'][:,0],label=legend)
            if len(data['u']): ax[2].plot(data['t'][:-1],np.max(data['u'],axis=1)/baseline_params()['n_max'],label=legend)
        for a,title in zip(ax,['Altitude error [m]','Forward speed error [m/s]','Max commanded rotor / limit']):
            a.set_ylabel(title); a.grid(alpha=.25); a.legend(fontsize=8)
        ax[2].axhline(1.,color='k',ls='--',lw=.7); ax[2].set_xlabel('Time [s]')
        fig.suptitle(f'Frozen pack_forward v2 | {speed:g} m/s | {case}'+(' | mission' if mission else ''))
        fig.tight_layout(); fig.savefig(out/f'{speed:g}_{case}_{"mission" if mission else "short"}.png',dpi=150)
        plt.close(fig)


def write_summary(out,meta):
    p=meta['params']
    lines=['# 고정 기체 제어기 사전 비교','',
           '| 기체 설정 | 값 |','|---|---|',
           f'| 모델 | {p["profile_id"]} |',f'| 질량 | {p["mass"]:.6f} kg |',
           f'| 동체 길이 / 직경 | {p["body_length"]:.3f} / {p["body_diameter"]:.3f} m |',
           '| 배터리 위치 | 동체 중앙에서 기수 쪽 0.020 m |',
           f'| 회전수 상한 / 모터 시정수 | {p["n_max"]*60/(2*np.pi):.0f} rpm / {1000*p["tau_m"]:.0f} ms |',
           '| 추진식 | APC Ct(J), Cp(J) 독립 PCHIP, 30k rpm 계수 형상 고정 |',
           '',f'파라미터 SHA256: `{PARAMETER_SHA256}`','',
           '중단 결과의 RMSE는 실패 전 부분 구간 값이며 완주 결과와 성능 순위를 비교하면 안 된다.',
           '완주와 추종 기준 통과는 구분한다. 추진 가정 범위 밖 체류율이 있으면 해당 구간의 물리적 해석에 한계가 있다.','',
           '| 속도 / 조건 | 제어기 | 고도 RMSE m | 최대 고도편차 m | 속도 norm RMSE m/s | 포화 % | 완료 / 추종 | 최적화 실패 | 범위 밖 % |',
           '|---|---|---:|---:|---:|---:|---|---:|---:|']
    for r in meta['results']:
        condition=r['case']+(' (65s mission)' if r['mission'] else '')
        state=('완주' if r['completed'] else f'중단 {r["elapsed_sim_s"]:.3f}s')+(' / 통과' if r['tracking_pass'] else ' / 미달')
        lines.append(f'| {r["speed_m_s"]:g} / {condition} | {r["controller"]} | {r["z_rmse_m"]:.4f} | '
                     f'{r["z_max_m"]:.3f} | {r["velocity_rmse_m_s"]:.3f} | {100*r["saturation_fraction"]:.2f} | '
                     f'{state} | {r["optimizer_failures"]}/{r["optimizer_calls"]} | {100*r["prop_domain_outside_fraction"]:.2f} |')
    lines+=['','## 중단·생략 사유','']
    for r in meta['results']:
        if r['stop_reason']:
            lines.append(f'- {r["controller"]} / {r["speed_m_s"]:g} / {r["case"]}: {r["stop_reason"]}')
    lines.extend('- '+s for s in meta['skipped'])
    lines+=['','상세 설정·솔버 이력·소스 해시는 report.json, 시계열은 각 NPZ, 대표 응답은 PNG에 저장했다.',
            'comparison_nmpc.py의 가상입력 비선형 제약을 사용한다. 옛 VirtualNMPC/acados 경로는 이 공유본에 포함하지 않는다.',
            '형상 탐색·기체 재튜닝·배터리/열/센서 모델·대규모 MC를 추가하지 않았다.']
    (out/'SUMMARY.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--stage',choices=['lqr','pilot','short','all'],default='short')
    parser.add_argument('--controllers',nargs='+',choices=LABELS,default=list(LABELS))
    args=parser.parse_args()
    out=ROOT/'results'/'baseline_v2'/('run_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    out.mkdir(parents=True,exist_ok=False)
    p=baseline_params(); factory=Factory(p)
    source={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (ROOT/'control').glob('*.py')}
    meta=dict(stage=args.stage,params=p,parameter_sha256=PARAMETER_SHA256,source_sha256=source,
              lqr_design=dict(Q=factory.Q,R=factory.R,grid=GRID,eigenvalues=factory.lqr_gains),
              settings=dict(dt_plant=DT,N=15,dt_prediction=.04,dt_outer=.04,max_iter=80),results=[],skipped=[])
    def save():
        (out/'report.json').write_text(json.dumps(jsonable(meta),ensure_ascii=False,indent=2),encoding='utf-8')
        write_summary(out,meta)
    def run(label,speed,case,mission=False):
        r=run_case(factory,label,speed,case,out,mission=mission)
        meta['results'].append(r); save(); return r
    print(f'Results: {out}',flush=True)
    smoke=run('GS-LQR',70.,'lqr_smoke')
    if not smoke['tracking_pass'] or smoke['z_max_m'] >= .5:
        meta['skipped'].append('LQR smoke gate failed; remaining experiments not run')
        save(); plots(out,meta['results']); return
    if args.stage!='lqr':
        cases=['nominal'] if args.stage=='pilot' else ['nominal','aero_plus','aero_minus','vertical','lateral']
        for case in cases:
            for label in args.controllers: run(label,70.,case)
    if args.stage=='all':
        eligible=[label for label in args.controllers if all(r['completed'] for r in meta['results']
                  if r['controller']==label and r['speed_m_s']==70.) and any(
                    r['controller']==label and r['case']=='nominal' and r['tracking_pass'] for r in meta['results'])]
        for label in args.controllers:
            if label not in eligible: meta['skipped'].append(f'{label}: high speed/mission skipped after short-test gate failure')
        for speed in (80.,85.):
            for case in ('nominal','vertical'):
                for label in eligible: run(label,speed,case)
        for label in eligible:
            run(label,70.,'legacy_vertical10',mission=True)
    plots(out,meta['results']); save()
    print(f'Saved: {out}',flush=True)


if __name__=='__main__': main()
