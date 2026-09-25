# Fixed aircraft distribution

This repository intentionally distributes only `light_rocket_v2_pack_forward`.
Read README.md and docs/INTEGRATION.md before changing or porting the model.

- The fixed parameter SHA-256 is `13a1dcc6332987eba93ea4653f4d6087741d1ac085701982ed94c22e6658c807`.
- Use `baseline_params()` for independent dictionaries; `vehicle_params` is only an import-name compatibility alias to the same aircraft.
- Do not reintroduce the 8 kg model, other battery candidates, old result sets, or old constant-Q/T VirtualNMPC as defaults.
- Preserve the 17-state ordering, body +x thrust, world z-up, quaternion xyzw/body-to-world, rad/s motor units and body reaction-torque signs.
- Port the dynamics, component aero, independent Ct/Cp propeller map, nonlinear allocation and control-effectiveness derivatives together. Changing mass alone is not an integration.
- The virtual-input optimizer is `ComparisonNMPC(virtual=True)`. Its auxiliary rotor variables certify static feasibility; only INDI outputs real motor commands. This does not certify motor-lag reachability.
- Physical motor bounds are 0..33000 rpm. The assumed data working range 10000..33000 rpm is not a new hardware minimum. Log model-domain excursions.
- Keep the archived 61-case result directory immutable. New experiments write new result folders. Do not claim partial runs are full missions or that all controllers succeed.
- Run `python -m control.validate_model --authority`, `python -m pytest tests -q`, and an appropriate short closed-loop check after integration changes.
- Do not change the fixed airframe to improve a controller's ranking. Any user-requested future model revision needs a new ID, hash, validation and separate results.
- No push to the upstream team repository. Remote writes require the user's request and an exact repository check.
- This repository omits battery electrical/thermal models, sensors/estimation, acados, ROS2/PX4, Simulink and real-flight integration. Do not silently add them during a model-porting task.
