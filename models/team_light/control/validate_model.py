"""Quick fixed-model validation: hash, mass properties, trim, static authority.

python -m models.team_light.control.validate_model
"""
import argparse
import numpy as np
from models.team_light.control.baseline_v2 import baseline_params, parameter_hash
from models.team_light.control.airframe import mass_properties
from models.team_light.control.dynamics import AxialDronePlant
from models.team_light.control.trim import find_trim
from models.team_light.control.propeller_curve import domain_status
from models.team_light.control.curve_authority import nonlinear_trim_authority


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--authority', action='store_true', help='Also check static 70/85 m/s margins.')
    args = parser.parse_args()
    p = baseline_params()
    mass, cg, inertia = mass_properties(p['mass_elements'])
    np.testing.assert_allclose([mass], [p['mass']], atol=1e-14)
    np.testing.assert_allclose(cg, p['cg_datum'], atol=1e-14)
    np.testing.assert_allclose(inertia, p['inertia_tensor'], atol=1e-14)
    plant = AxialDronePlant(p, dt=.002)
    print(p['profile_id'])
    print('Parameter SHA256:', parameter_hash(p))
    print('Mass:', mass, 'kg; CG:', cg, 'm; inertia:', np.diag(inertia), 'kg m^2')
    print('speed m/s | pitch deg | max rpm | upper RPM margin | residual')
    for speed in (0., 10., 20., 30., 40., 50., 60., 70., 75., 80., 85.):
        tr = find_trim(p, speed)
        assert domain_status(p, tr['control'], tr['v_body'])['inside_assumed_domain']
        xd = plant.evaluate_xdot(tr['state'], tr['control'])
        np.testing.assert_allclose(xd[3:], 0., atol=1e-8)
        next_state = plant.step(tr['state'], tr['control'])
        np.testing.assert_allclose(next_state[3:], tr['state'][3:], atol=1e-8)
        print(f"{speed:5.1f} | {np.degrees(tr['theta']):9.3f} | "
              f"{np.max(tr['control'])*60/(2*np.pi):8.1f} | "
              f"{100*(1-np.max(tr['control'])/p['n_max']):8.2f}% | {tr['residual']:.2e}")
        if args.authority and speed in (70., 85.):
            margin = nonlinear_trim_authority(p, tr)
            assert margin['valid'] and margin['passes_predeclared_margin'], margin
            print('  Minimum bidirectional static angular-acceleration margin:',
                  np.min(margin['angular_accel_margin_rad_s2'], axis=1))
    print('PASS: numerical consistency and sampled trims; NOT real-flight validation.')


if __name__ == '__main__':
    main()
