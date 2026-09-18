"""Compile CasADi scalar instructions into JS, not a second set of physics.

The bundled WASM API creates many matrix proxies per 1 ms tick. Plain numeric
evaluators avoid that boundary overhead for the plant/estimator/allocator only.
IPOPT and its prediction/derivative graphs remain in CasADi/WASM.
Unsupported graph operations stop the build instead of silently approximating.
"""
import json
import math
import casadi as ca
from research.model import _interpolator


def number(value):
    value = float(value)
    if math.isnan(value):
        return "NaN"
    if math.isinf(value):
        return "Infinity" if value > 0 else "-Infinity"
    return repr(value)


def compile_numeric(functions, estimator, profile):
    tables = [(profile["aero"]["speed_mps"], profile["aero"]["CD0"]),
              ([0, .5, 1], [3.4, 3.7, 4.2])]
    if not profile["prop"].get("simple"):
        tables += [(profile["prop"]["J"], profile["prop"][key]) for key in ("CT", "CP")]
    lookup = {}
    for grid, values in tables:
        fn = _interpolator(tuple(grid), tuple(values))
        lookup[fn.serialize()] = (grid, values, False)
        lookup[fn.jacobian().serialize()] = (grid, values, True)
    definitions, names = [], {}
    binary = {ca.OP_ADD: "+", ca.OP_SUB: "-", ca.OP_MUL: "*", ca.OP_DIV: "/"}
    unary = {ca.OP_SQRT: "sqrt", ca.OP_SIN: "sin", ca.OP_COS: "cos", ca.OP_FABS: "abs"}

    def emit(fn):
        serialized = fn.serialize()
        if serialized in names:
            return names[serialized]
        name = "f" + str(len(names))
        names[serialized] = name
        if serialized in lookup:
            grid, values, derivative = lookup[serialized]
            definitions.append(f"function {name}(a){{return [lookup(a[0],{json.dumps(grid)},"
                               f"{json.dumps(values)},{str(derivative).lower()})];}}")
            return name
        if fn.class_name() in ("LinearInterpolant", "LinearInterpolantJac"):
            raise ValueError("Unregistered interpolation table: " + fn.name())
        f = fn.expand()
        sx = f.instructions_sx()
        in_offset, out_offset = [0], [0]
        for i in range(f.n_in()):
            in_offset.append(in_offset[-1] + f.nnz_in(i))
        for i in range(f.n_out()):
            out_offset.append(out_offset[-1] + f.nnz_out(i))
        body = [f"function {name}(a){{const w=new Float64Array({f.sz_w()}),o=new Array({out_offset[-1]});"]
        for i in range(f.n_instructions()):
            op, inputs, outputs = f.instruction_id(i), f.instruction_input(i), f.instruction_output(i)
            args = [f"w[{v}]" if v >= 0 else "0" for v in inputs]
            if op == ca.OP_INPUT:
                expression = f"a[{in_offset[inputs[0]] + inputs[1]}]"
            elif op == ca.OP_OUTPUT:
                body.append(f"o[{out_offset[outputs[0]] + outputs[1]}]={args[0]};")
                continue
            elif op == ca.OP_CONST:
                expression = number(f.instruction_constant(i))
            elif op == ca.OP_CALL:
                called = sx[i].which_function()
                callee = emit(called)
                body.append(f"{{const r={callee}([{','.join(args)}]);")
                for j, output in enumerate(outputs):
                    if output >= 0:
                        body.append(f"w[{output}]=r[{j}];")
                body.append("}")
                continue
            elif op in binary:
                expression = f"{args[0]}{binary[op]}{args[1]}"
            elif op in unary:
                expression = f"Math.{unary[op]}({args[0]})"
            elif op == ca.OP_NEG:
                expression = "-" + args[0]
            elif op == ca.OP_SQ:
                expression = f"{args[0]}*{args[0]}"
            elif op in (ca.OP_FMIN, ca.OP_FMAX):
                expression = f"{'fmin' if op == ca.OP_FMIN else 'fmax'}({args[0]},{args[1]})"
            elif op in (ca.OP_LT, ca.OP_LE):
                expression = f"({args[0]}{'<' if op == ca.OP_LT else '<='}{args[1]}?1:0)"
            elif op in (ca.OP_AND, ca.OP_OR):
                expression = f"({args[0]}{'&&' if op == ca.OP_AND else '||'}{args[1]}?1:0)"
            elif op == ca.OP_NOT:
                expression = f"({args[0]}?0:1)"
            elif op == ca.OP_IF_ELSE_ZERO:
                expression = f"({args[0]}?{args[1]}:0)"
            else:
                raise ValueError(f"Unsupported CasADi operation {op} in {fn.name()}")
            body.append(f"w[{outputs[0]}]={expression};")
        body.append("return o;}")
        definitions.append("\n".join(body))
        return name

    exported = {}
    for group, entries in (("functions", functions), ("estimator", estimator)):
        for key, fn in entries.items():
            # Optimizer graphs stay in WASM. Export only the time-step hot path.
            if group == "functions" and key not in ("rhs", "step", "diag", "aero", "rotors", "inverse_thrust", "motor_step"):
                continue
            name = emit(fn)
            exported[group + "." + key] = {"name": name,
                "inputs": [list(fn.sparsity_in(i).find()) for i in range(fn.n_in())],
                "outputs": [{"size": fn.numel_out(i), "indices": list(fn.sparsity_out(i).find())}
                            for i in range(fn.n_out())]}
    preamble = """
const fmin=(a,b)=>Number.isNaN(a)?b:Number.isNaN(b)?a:Math.min(a,b);
const fmax=(a,b)=>Number.isNaN(a)?b:Number.isNaN(b)?a:Math.max(a,b);
function lookup(x,grid,values,derivative){
  let i=0;while(i<grid.length-2&&x>=grid[i+1])i++;
  const slope=(values[i+1]-values[i])/(grid[i+1]-grid[i]);
  if(derivative)return slope;
  const alpha=(x-grid[i])/(grid[i+1]-grid[i]);
  return (1-alpha)*values[i]+alpha*values[i+1];
}
"""
    source = preamble + "\n".join(definitions) + "\nreturn {" + ",".join(
        json.dumps(key) + ":" + entry["name"] for key, entry in exported.items()) + "};"
    return {"format": "casadi-scalar-js-v1", "source": source, "layout": exported}
