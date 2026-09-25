# Team aircraft import

Source: https://github.com/KYUHYUNKANG03/fastdrone_kh

Branch: `repro/kyuhyun`; commit: `34981c5d64c102df5c2fd8b4b3d9a5e0e3db81f7`.

Model: `light_rocket_v2_pack_forward`, 1.17407572 kg.
Nominal parameter SHA-256:
`13a1dcc6332987eba93ea4653f4d6087741d1ac085701982ed94c22e6658c807`.

The Python model, control adapters, data and acceptance tests are imported as
`models.team_light.control` so the original `control` branch implementations
remain available. Changes to upstream source are limited to Python import paths,
module invocation examples and the self-contained import test. Dynamics, curves,
controller settings and the frozen parameter hash are unchanged.

`upstream_hashes.json` records original file hashes. `UPSTREAM_NOTICE.md` preserves
attribution. `UPSTREAM_README.md` and `docs/INTEGRATION.md` describe the source
model's assumptions and integration contract. One archived GS-LQR JSON is retained
solely as the upstream regression fixture; new runs must not overwrite it.

Run from the project root:

```sh
python -m models.team_light.control.validate_model --authority
python -m pytest models/team_light/tests -q
```

New scenarios, acceptance criteria and uncertainty sampling live in the parent
project's `control/validation_suite.py`, `mission_profiles.py`,
`validation_metrics.py` and `uncertainty.py`. Plant perturbations are explicitly
labelled trial copies and never change the nominal aircraft factory or its hash.
