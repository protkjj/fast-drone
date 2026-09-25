"""Acceptance checks for the single-aircraft distribution, not flight validation."""
import importlib
import json
from pathlib import Path
import casadi as ca
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from models.team_light.control.airframe import mass_properties
from models.team_light.control.baseline_v2 import baseline_params, parameter_hash, PARAMETER_SHA256
from models.team_light.control.light_rocket_curve import make_curve_rocket
from models.team_light.control.serialization import jsonable
from models.team_light.control.dynamics import AxialDronePlant, _body_aerodynamics, _rotor_forces_moments, compute_allocation_matrix
from models.team_light.control.geometry import (rotor_thrusts, rotor_wrench, rotor_speeds_for_thrust,
                              control_effectiveness, allocate_wrench, hover_quaternion)
from models.team_light.control.propeller_curve import coefficients, values_and_derivatives, symbolic_force_torque, domain_status
from models.team_light.control.trim import find_trim
from models.team_light.control.curve_authority import nonlinear_trim_authority

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def p():
    return baseline_params()


def test_only_one_default_and_independent_dictionaries(p):
    from models.team_light.control.vehicle_params import vehicle_params
    snapshot = json.loads((ROOT/'data/aircraft_snapshot.json').read_text(encoding='utf-8'))
    assert jsonable(p) == snapshot
    assert parameter_hash(vehicle_params) == PARAMETER_SHA256
    assert parameter_hash(make_curve_rocket()) == PARAMETER_SHA256
    with pytest.raises(ValueError, match='only'):
        make_curve_rocket('unsupported_candidate')
    p['rotor_positions'][0, 0] += 1.
    assert parameter_hash(baseline_params()) == PARAMETER_SHA256


def test_component_mass_cg_and_inertia(p):
    mass, cg, inertia = mass_properties(p['mass_elements'])
    assert mass == pytest.approx(1.17407572)
    np.testing.assert_allclose(cg, p['cg_datum'])
    np.testing.assert_allclose(inertia, p['inertia_tensor'])
    assert np.min(np.linalg.eigvalsh(inertia)) > 0.
    battery = next(e for e in p['mass_elements'] if e['name']=='battery_mass_only')
    assert battery['position'] == [.02, 0, 0]
    assert p['body_length'] == .32 and p['body_diameter'] == .075
    assert p['tau_m'] == .020
    assert p['n_max']*60/(2*np.pi) == pytest.approx(33000.)


def test_hover_frame_and_force_torque_signs(p):
    plant = AxialDronePlant(p)
    x = plant.hover_state(p)
    np.testing.assert_allclose(Rotation.from_quat(hover_quaternion()).apply([1.,0,0]), [0,0,1], atol=1e-12)
    np.testing.assert_allclose(plant.evaluate_xdot(x, x[13:17]), 0., atol=1e-10)
    f, m = _rotor_forces_moments(ca.DM.zeros(3), ca.DM([1500.,0,0,0]), ca.DM.zeros(3), p)
    assert float(f[0]) > 0. and float(m[0]) > 0.
    np.testing.assert_allclose(np.array(ca.DM(f)).reshape(3)[1:], 0., atol=1e-12)
    free = x.copy(); free[13:17] = 0.
    np.testing.assert_allclose(plant.evaluate_xdot(free, np.zeros(4))[3:6], [0,0,-p['g']], atol=1e-10)


@pytest.mark.parametrize('speed', [0., 70., 80., 85.])
def test_transfer_trim_fixtures(p, speed):
    fixture = json.loads((ROOT/'data/acceptance_fixtures.json').read_text(encoding='utf-8'))
    assert fixture['parameter_sha256'] == PARAMETER_SHA256
    expected = fixture['trims'][str(speed)]
    tr = find_trim(p, speed)
    assert tr['valid']
    np.testing.assert_allclose(tr['state'], expected['state'], rtol=1e-8, atol=1e-8)
    np.testing.assert_allclose(tr['control'], expected['control'], rtol=1e-8, atol=1e-8)
    assert domain_status(p, tr['control'], tr['v_body'])['inside_assumed_domain']
    xd = AxialDronePlant(p).evaluate_xdot(tr['state'], tr['control'])
    np.testing.assert_allclose(xd[:3], [speed,0,0], atol=1e-9)
    np.testing.assert_allclose(xd[3:], 0., atol=1e-8)


@pytest.mark.parametrize('wind', [[5.,0,0], [0,5.,0], [0,0,5.]])
def test_wind_is_world_air_relative_velocity(p, wind):
    plant = AxialDronePlant(p)
    x = find_trim(p, 70.)['state'].copy()
    relative = x.copy(); relative[3:6] -= wind
    xd = plant.evaluate_xdot(x, x[13:17], wind)
    np.testing.assert_allclose(xd[3:], plant.evaluate_xdot(relative, x[13:17])[3:], atol=1e-10)
    np.testing.assert_allclose(xd[:3], x[3:6], atol=1e-10)


@pytest.mark.parametrize('velocity', [[40.,3.,6.], [-20.,1.,-3.], [0.,0.,0.]])
def test_component_aero_dissipates_energy(p, velocity):
    omega = np.array([.4,-.2,.3])
    f, m = _body_aerodynamics(ca.DM(velocity), ca.DM(omega), p)
    assert float(ca.dot(f,ca.DM(velocity))+ca.dot(m,ca.DM(omega))) <= 1e-10


def test_apc_anchor_and_torque_units(p):
    rows = np.asarray(p['prop_curve']['knots'])
    np.testing.assert_allclose(np.column_stack(coefficients(p, rows[:,0])), np.maximum(rows[:,1:],0.), atol=1e-14)
    rate = 30000.*2*np.pi/60
    thrust, torque, _, _ = values_and_derivatives(p, [rate], .7908*500*p['D_prop'])
    assert thrust[0] == pytest.approx(14.725, rel=.002)
    assert torque[0] == pytest.approx(.343, rel=.004)
    static = values_and_derivatives(p, [rate], 0.)
    assert not np.isclose(static[1][0]/static[0][0], torque[0]/thrust[0], rtol=.01)


@pytest.mark.parametrize('va', [0., 50., 85., 110.])
def test_numeric_symbolic_and_finite_difference(p, va):
    rates = np.array([1000.,2300.,3100.,3400.])
    rate = ca.SX.sym('rate')
    thrust, torque = symbolic_force_torque(p, rate, va)
    f = ca.Function('prop_check', [rate], [ca.vertcat(thrust,torque,ca.jacobian(thrust,rate),ca.jacobian(torque,rate))])
    expected = np.asarray(values_and_derivatives(p, rates, va))
    np.testing.assert_allclose(np.column_stack([np.array(f(n)).reshape(4) for n in rates]), expected, rtol=1e-11, atol=1e-11)
    plus = np.array(values_and_derivatives(p,rates+.001,va)[:2])
    minus = np.array(values_and_derivatives(p,rates-.001,va)[:2])
    np.testing.assert_allclose(expected[2:], (plus-minus)/.002, rtol=1e-6, atol=1e-8)


@pytest.mark.parametrize('va', [0., 70., 85.])
def test_inverse_allocation_and_effectiveness(p, va):
    rates = np.array([3010.,3050.,3190.,3270.])
    vb = [va,2.,3.]
    thrust = rotor_thrusts(p,rates,vb)
    np.testing.assert_allclose(rotor_speeds_for_thrust(p,thrust,vb),rates,atol=1e-8)
    target = rotor_wrench(p,rates,vb)
    commands = allocate_wrench(p,target,vb)
    assert np.all((commands>=0.) & (commands<=p['n_max']))
    np.testing.assert_allclose(rotor_wrench(p,commands,vb),target,atol=1e-6)
    inertia = np.array([p['Ixx'],p['Iyy'],p['Izz']])
    def response(n):
        force,moment = _rotor_forces_moments(ca.DM(vb),ca.DM(n),ca.DM.zeros(3),p)
        return np.r_[float(force[0]),np.array(ca.DM(moment)).reshape(3)/inertia]
    fd = np.column_stack([(response(rates+e*.001)-response(rates-e*.001))/.002 for e in np.eye(4)])
    np.testing.assert_allclose(control_effectiveness(p,rates,vb),fd,rtol=1e-6,atol=1e-7)
    with pytest.raises(ValueError, match='Variable Q/T'):
        compute_allocation_matrix(p)


def test_static_authority_at_85(p):
    tr = find_trim(p,85.)
    margin = nonlinear_trim_authority(p,tr)
    assert margin['valid'] and margin['passes_predeclared_margin']


def test_lqr_reproduces_archived_nominal(p,tmp_path):
    from models.team_light.control.run_baseline_comparison import Factory,run_case
    archived = ROOT/'results/baseline_v2/run_20260924T161648152853Z/GS-LQR_70_nominal.json'
    old = json.loads(archived.read_text(encoding='utf-8'))
    result = run_case(Factory(p),'GS-LQR',70.,'nominal',tmp_path)
    assert result['completed'] and result['tracking_pass']
    for key in ('z_rmse_m','z_max_m','velocity_rmse_m_s','saturation_fraction'):
        assert result[key] == pytest.approx(old[key], rel=1e-8, abs=1e-9)


def test_imports_are_self_contained():
    import models.team_light.control as control
    assert Path(control.__file__).resolve().parent == ROOT/'control'
    for path in (ROOT/'control').glob('*.py'):
        if path.stem != '__init__':
            importlib.import_module('models.team_light.control.'+path.stem)
