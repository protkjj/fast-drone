"""Frozen-model and nonlinear virtual-command feasibility regression tests."""
import casadi as ca
import numpy as np
import pytest

from models.team_light.control.baseline_v2 import baseline_params,parameter_hash,PARAMETER_SHA256
from models.team_light.control.comparison_nmpc import ComparisonNMPC,virtual_wrench_residual,positive_thrust_domain
from models.team_light.control.dynamics import AxialDronePlant
from models.team_light.control.hybrid_comparison import build_virtual_dynamics,ProperHybrid
from models.team_light.control.geometry import rotor_thrusts
from models.team_light.control.trim import find_trim


def test_aircraft_lock():
    p=baseline_params()
    assert p['profile_id']=='light_rocket_v2_pack_forward'
    assert parameter_hash(p)==PARAMETER_SHA256
    p['mass']+=.01
    assert parameter_hash(p)!=PARAMETER_SHA256


@pytest.mark.parametrize('speed',[0.,70.,85.])
def test_nonlinear_witness_matches_full_dynamics(speed):
    p=baseline_params(); tr=find_trim(p,speed)
    x=tr['state'].copy(); x[10:13]=[.2,-.15,.1]
    x[13:17]*=np.array([1.001,.999,1.002,.998])
    plant=AxialDronePlant(p)
    xd=plant.evaluate_xdot(x,x[13:17])
    virtual=np.r_[np.sum(rotor_thrusts(p,x[13:17],tr['v_body'])),xd[10:13]]
    residual=virtual_wrench_residual(ca.DM(x[:13]),ca.DM(virtual),ca.DM(x[13:17]),p)
    np.testing.assert_allclose(np.array(ca.DM(residual)).reshape(4),0.,atol=1e-10)
    fv,_,_=build_virtual_dynamics(p)
    np.testing.assert_allclose(np.array(fv(x[:13],virtual)).reshape(13),xd[:13],atol=1e-10)
    assert np.min(np.array(ca.DM(positive_thrust_domain(ca.DM(x),ca.DM(x[13:17]),p))))>=0.
    # Old static-Q/T proxy cannot certify an arbitrarily changed command.
    bad=virtual.copy(); bad[1]+=100.
    assert np.linalg.norm(np.array(ca.DM(virtual_wrench_residual(ca.DM(x[:13]),ca.DM(bad),ca.DM(x[13:17]),p))))>.1


@pytest.mark.parametrize('speed',[70.,85.])
@pytest.mark.parametrize('virtual',[False,True])
def test_solver_solution_obeys_actual_nonlinear_envelope(speed,virtual):
    p=baseline_params(); tr=find_trim(p,speed)
    x=tr['state'].copy(); x[2]=20.
    ctrl=ComparisonNMPC(p,tr,virtual=virtual,z_ref=20.,N=3)
    command=ctrl(0.,x)
    assert ctrl.solve_log[-1]['accepted'],ctrl.solve_log[-1]
    assert ctrl.solve_log[-1]['scaled_constraint_violation']<1e-5
    answer=ctrl.last_solution
    if virtual:
        rates=answer[4:8]*p['n_max']
        assert np.all((rates>=p['n_min']) & (rates<=p['n_max']))
        residual=virtual_wrench_residual(ca.DM(x[:13]),ca.DM(command),ca.DM(rates),p)
        np.testing.assert_allclose(np.array(ca.DM(residual)),0.,atol=1e-6)
        # A motor command is produced by the inner loop, NOT auxiliary rates.
        inner=ProperHybrid(ctrl,p,dt=.002)
        np.testing.assert_allclose(inner(0.,x),x[13:17],atol=1e-10)
    else:
        assert np.all((command>=p['n_min']) & (command<=p['n_max']))
    assert parameter_hash(p)==PARAMETER_SHA256
    ctrl.reset(); assert not ctrl.solve_log and ctrl.consec_fail==0
