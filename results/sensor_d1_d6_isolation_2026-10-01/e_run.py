"""D1·D6 isolation: run one smoke trial of gust_lateral_p10_VL on the v9 sensor config.

Usage: python3 e_run.py <tree_root> <variant> <controller> <seed> <patch> <out_dir>
  variant: full | quiet_sampled | imu_only | navigation_only | rotor_only
  patch:   none | fallback_hold  (E4: V13/F13 first-call fallback command -> hold observed rotor speed)
Nothing inside <tree_root> is modified; the derived config and all outputs go to <out_dir>.
"""
import json
import os
import sys


def main():
    root, variant, ctrl, seed, patch, out = sys.argv[1:7]
    sys.path.insert(0, root)
    os.chdir(root)
    os.makedirs(out, exist_ok=True)
    from control.arena import load_config, validate_config
    from control.arena_sensors import load_sensor_profile
    from control.sensor_binding import resolve_feedback
    from control.sensor_matching_screen import sensor_group_profile

    from copy import deepcopy
    config = load_config('configs/arena_rotor_projected_development_v9.json')
    nominal = resolve_feedback(config).profile
    # E5: '<group>_sig005' = same group with estimator initial position sigma 0.05 m (was 0.5 m).
    # E2: 'baro_only' / 'gnss_only' = quiet_sampled with only that sensor's errors restored.
    sigma = None
    group = variant
    if variant.endswith('_sig005'):
        group, sigma = variant[:-len('_sig005')], 0.05
    if group != 'full' or sigma is not None:
        if group == 'full':
            profile = deepcopy(dict(nominal))
        elif group in ('baro_only', 'gnss_only'):
            profile = dict(sensor_group_profile(nominal, 'quiet_sampled'))
            sensor = 'barometer' if group == 'baro_only' else 'gnss'
            profile[sensor] = deepcopy(nominal[sensor])
        else:
            profile = dict(sensor_group_profile(nominal, group))
        if sigma is not None:
            profile['estimator'] = dict(profile['estimator'], initial_position_sigma=sigma)
        profile['name'] = f'{variant}_nominal'
        config['sensor_feedback']['profile'] = load_sensor_profile(profile)
        config = validate_config(config)
    config_path = os.path.join(out, 'config.json')
    with open(config_path, 'w', encoding='utf-8') as f:
        json.dump(config, f, ensure_ascii=False, indent=1, allow_nan=False)

    if patch == 'fallback_hold':
        import numpy as np
        from control.hybrid_comparison import ProperHybrid
        original = ProperHybrid.__call__

        def patched(self, t, x):
            first = not self._initialized
            u = original(self, t, x)
            if first and isinstance(self.probe, dict) and self.probe.get('path') == 'fallback_init':
                self.probe['path'] = 'fallback_init_hold'
                return np.clip(np.asarray(x, dtype=float)[13:17], self.p['n_min'], self.p['n_max'])
            return u
        ProperHybrid.__call__ = patched
    elif patch != 'none':
        raise SystemExit(f'unknown patch {patch}')

    from control.validation_suite import main as suite_main
    return suite_main(['--config', config_path, '--smoke', '--only-cases', 'gust_lateral_p10_VL',
                       '--only-controllers', ctrl, '--sensor-seed', str(seed), '--output', out])


if __name__ == '__main__':
    sys.exit(main())
