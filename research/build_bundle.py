"""Build platform-neutral CasADi functions; no solver runs on the web server."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import casadi as ca
import numpy as np
from research import eskf
from research.model import profile, build, initial_state
from research.nmpc import build_solver


def bundle(name, include_solvers=True):
    p = profile(name)
    functions = build(p)
    estimators = eskf.build(p["g"])
    x0 = initial_state(p)
    payload = {"schema": 1, "casadi_python_version": ca.__version__, "casadi_wasm_version": "3.8.0",
               "profile": p, "sensors": eskf.SENSORS, "initial": x0.tolist(),
               "functions": {k: v.serialize() for k,v in functions.items()},
               "estimator": {k: v.serialize() for k,v in estimators.items()}, "solvers": {}}
    if include_solvers:
        for kind in ("hybrid", "nmpc"):
            begin = time.perf_counter()
            solver, meta = build_solver(p, functions, kind)
            payload["solvers"][kind] = {"function": solver.serialize(), **meta}
            print(f"{name}/{kind} built in {time.perf_counter()-begin:.2f} s", flush=True)
    # Fixtures catch cross-language/serializer errors with actual numerical values.
    x = x0.copy()
    x[3:6] = [8, 1, 2]
    x[10:13] = [.1, -.2, .3]
    u, env, scales = (x[13:17]*[1.05,.98,1.1,1]).tolist(), [2,0,0,.01,0,0], [1.1,1,.9,1.1,.95]
    payload["parity"] = {"x": x.tolist(), "u": u, "env": env, "scales": scales,
                         "rhs": np.asarray(functions["rhs"](x,u,env,scales)).ravel().tolist(),
                         "step": np.asarray(functions["step"](x,u,env,scales)).ravel().tolist()}
    return payload


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", choices=["simple", "selected", "all"], default="all")
    ap.add_argument("--output-dir", type=Path, default=Path(__file__).parent/"generated")
    ap.add_argument("--without-solvers", action="store_true")
    args = ap.parse_args()
    args.output_dir.mkdir(exist_ok=True, parents=True)
    for name in (["simple", "selected"] if args.profile == "all" else [args.profile]):
        output = args.output_dir / f"{name}.json"
        text = json.dumps(bundle(name, not args.without_solvers), separators=(",", ":"), ensure_ascii=False)
        output.write_text(text)
        print(f"{output.name}: {len(text)/1e6:.2f} MB; sha256={hashlib.sha256(text.encode()).hexdigest()}", flush=True)
