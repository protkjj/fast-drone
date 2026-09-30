"""Compare evaluation-0 results of two trees: objective and every scenario field must be identical.

Usage: python3 baseline_compare.py <base_json> <sensor_json>
"""
import json
import sys


def main():
    a, b = (json.load(open(p, encoding='utf-8')) for p in sys.argv[1:3])
    problems = []
    for key in ('label', 'config_sha256', 'controller_model_sha256', 'parameter_names', 'prior'):
        if a[key] != b[key]:
            problems.append(f'{key} differs')
    if [s['id'] for s in a['scores']] != [s['id'] for s in b['scores']]:
        problems.append('scenario ids/order differ')
    fields = []
    for sa, sb in zip(a['scores'], b['scores']):
        for key in sorted(set(sa) | set(sb)):
            if sa.get(key) != sb.get(key):
                fields.append((sa['id'], key))
    traj_same = sum(sa.get('trajectory_sha256') == sb.get('trajectory_sha256')
                    for sa, sb in zip(a['scores'], b['scores']))
    bit = a['objective'] == b['objective'] and not fields and not problems
    print(json.dumps(dict(label=a['label'], base_head=a['git_head'], sensor_head=b['git_head'],
                          config_sha256=a['config_sha256'], objective_base=a['objective'],
                          objective_sensor=b['objective'], trajectories_identical=f'{traj_same}/{len(a["scores"])}',
                          differing_fields=fields, problems=problems, bit_identical=bit,
                          wall_s=[a['wall_s'], b['wall_s']]), ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
