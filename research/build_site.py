"""Package the optional research page with pinned, same-origin WASM dependencies."""
import argparse
import gzip
import hashlib
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent


def build(output):
    package = ROOT / "node_modules/@casadi/casadi-wasm"
    files = ["casadi.js", "casadi_wasm.js", "casadi_wasm.wasm",
             "libcasadi_nlpsol_ipopt.so", "libcasadi_interpolant_linear.so"]
    for name in files:
        if not (package / name).is_file():
            raise FileNotFoundError(f"Missing pinned WASM dependency: {name}; run npm ci in research/")
    for name in ("simple", "selected"):
        if not (ROOT / f"generated/{name}.json").is_file():
            raise FileNotFoundError("Run python3 -m research.build_bundle first")
    output.mkdir(parents=True, exist_ok=True)
    sources=("index.html", "classic.css", "page.js", "worker.js", "runtime.js", "stl.js", "results.js")
    version=hashlib.sha256(b"".join((ROOT/name).read_bytes() for name in
        (*sources,"assets/selected_geometry.json","generated/simple.json","generated/selected.json"))).hexdigest()[:16]
    # HTML, worker, runtime and serialized functions form one compatible unit.
    # Cache-bust their references together so a returning tab cannot mix versions.
    for name in sources:
        text=(ROOT/name).read_text()
        for script in ("classic.css","page.js","worker.js","runtime.js","stl.js","results.js","assets/selected_geometry.json"):
            text=text.replace(f"./{script}'",f"./{script}?v={version}'")
            text=text.replace(f'./{script}"',f'./{script}?v={version}"')
        if name=="worker.js":
            text=text.replace(".json.gz`",f".json.gz?v={version}`")
        (output/name).write_text(text)
    (output / "assets").mkdir(exist_ok=True)
    shutil.copy2(ROOT / "assets/drone_v2.stl", output / "assets/drone_v2.stl")
    shutil.copy2(ROOT / "assets/selected_geometry.json", output / "assets/selected_geometry.json")
    (output / "generated").mkdir(exist_ok=True)
    for name in ("simple", "selected"):
        data = (ROOT / f"generated/{name}.json").read_bytes()
        compressed = gzip.compress(data, compresslevel=9, mtime=0)
        (output / f"generated/{name}.json.gz").write_bytes(compressed)
        print(f"{name}: {len(data)/1e6:.2f} MB -> {len(compressed)/1e6:.2f} MB transfer")
    vendor = output / "vendor/casadi"
    vendor.mkdir(parents=True, exist_ok=True)
    for name in files:
        shutil.copy2(package / name, vendor / name)
    shutil.copytree(package / "licenses", vendor / "licenses", dirs_exist_ok=True)
    print(f"Research page: {output}/index.html (asset version {version})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, default=ROOT.parent/"site/research")
    build(ap.parse_args().output)
