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
from research.numeric_export import compile_numeric
from research.gain_schedule import build_schedule


def bundle(name, include_solvers=True):
    p = profile(name)
    functions = build(p)
    estimators = eskf.build(p["g"])
    x0 = initial_state(p)
    payload = {"schema": 1, "casadi_python_version": ca.__version__, "casadi_wasm_version": "3.8.0",
               "profile": p, "sensors": eskf.SENSORS, "initial": x0.tolist(),
               "functions": {k: v.serialize() for k,v in functions.items()},
               "estimator": {k: v.serialize() for k,v in estimators.items()}, "solvers": {}}
    # Both profiles read selected.json; model.profile() applies the explicit
    # simple-model overrides. Hash those two sources, not a nonexistent file.
    sources={file:hashlib.sha256((Path(__file__).parent/file).read_bytes()).hexdigest()
             for file in ("model.py","nmpc.py","eskf.py","runtime.js","numeric_export.py","profiles/selected.json","../control/dynamics.py")}
    payload["provenance"]={"source_sha256":sources,
        "input_sha256":hashlib.sha256(json.dumps(sources,sort_keys=True).encode()).hexdigest()}
    payload["numeric"] = compile_numeric(functions, estimators, p)
    payload["provenance"]["numeric_backend"] = "casadi-scalar-js-v1"
    payload["provenance"]["numeric_source_sha256"] = hashlib.sha256(payload["numeric"]["source"].encode()).hexdigest()
    # GSLQR(표5) 이득 스케줄 — 이 프로파일은 20-80 m/s에 수평 트림이 없으므로
    # (research/TRIM_ENVELOPE_AUDIT.md) 0-18 m/s만 스케줄링한다. runtime.js의
    # GSLQR 클래스가 이 표를 속도로 선형보간한다(식42). "simple" 프로파일은
    # 테스트용 축소 물리 모델이라 트림 특성이 달라 같은 표를 쓰지 않는다.
    payload["gslqr"] = build_schedule(p, speeds=[0, 2, 4, 6, 8, 10, 12, 14, 16, 18])
    if include_solvers:
        for kind in ("hybrid", "nmpc", "f13"):
            begin = time.perf_counter()
            solver, meta = build_solver(p, functions, kind)
            payload["solvers"][kind] = {"function": solver.serialize(), **meta}
            print(f"{name}/{kind} built in {time.perf_counter()-begin:.2f} s", flush=True)
    # Fixtures catch cross-language/serializer errors with actual numerical values.
    x = x0.copy()
    x[3:6] = [8, 1, 2]
    x[10:13] = [.1, -.2, .3]
    u, env, scales = (x[13:17]*[1.05,.98,1.1,1]).tolist(), [2,0,0,.01,0,0], [1.1,1,.9,1.1,.95,1.05]
    payload["parity"] = {"x": x.tolist(), "u": u, "env": env, "scales": scales,
                         "rhs": np.asarray(functions["rhs"](x,u,env,scales)).ravel().tolist(),
                         "step": np.asarray(functions["step"](x,u,env,scales)).ravel().tolist(),
                         "motor_step": np.asarray(functions["motor_step"](x[13:17],u,2,22)).ravel().tolist()}
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
