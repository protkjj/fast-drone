"""Evaluate one controller's tuning prior (evaluation 0) on truth config in a given source tree.

Usage: python3 baseline_eval0.py <tree_root> <label> <scenario_workers> <out_json>
Imports come from <tree_root>, so the same script runs our 6cad076 and the sensor repo a44c718.
Nothing is written inside the tree (PYTHONDONTWRITEBYTECODE=1 is expected from the caller).
"""
import json
import os
import subprocess
import sys
import time


def main():
    root, label, workers, out = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
    sys.path.insert(0, root)
    os.chdir(root)
    from control.arena import load_config, config_sha256, DEFAULT_CONFIG
    from control.arena_factory import ArenaFactory, ControllerModel
    from control.arena_tune import Evaluator, parameter_space
    from control.validation_suite import json_safe
    from models.team_light.control.baseline_v2 import baseline_params

    config = load_config(DEFAULT_CONFIG)
    native = baseline_params()
    model = ControllerModel(native)
    names, prior, apply = parameter_space(config, label, ArenaFactory(config, native, model=model).gains)
    evaluator = Evaluator(config, native, model, label, scenario_workers=workers)
    started = time.time()
    try:
        objective, scores = evaluator(apply(prior))
    finally:
        evaluator.close()
    try:
        tree_id = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        tree_id = None
    doc = dict(tree=root, git_head=tree_id, label=label, config=str(DEFAULT_CONFIG),
               config_sha256=config_sha256(config), controller_model_sha256=model.sha256,
               parameter_names=names, prior=prior, scenario_workers=workers,
               objective=objective, scores=json_safe(scores), wall_s=round(time.time()-started, 1))
    with open(out, 'w', encoding='utf-8') as f:
        json.dump(doc, f, ensure_ascii=False, allow_nan=False, indent=1)
    print(f'{label} {root}: objective={objective!r} scenarios={len(scores)} wall={doc["wall_s"]}s', flush=True)


if __name__ == '__main__':
    main()
