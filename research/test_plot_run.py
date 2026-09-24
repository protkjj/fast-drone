"""plot_run.py 회귀 — 합성 CSV로 tilt 계산과 PNG 생성만 확인한다(matplotlib
렌더링 자체를 픽셀 단위로 검증하지 않음, 회귀 목적)."""
import numpy as np
import pandas as pd
import pytest

from research.plot_run import plot_run, tilt_from_vertical_deg


def test_tilt_matches_known_quaternions():
    # 호버(기수 +z, [0,-sqrt.5,0,sqrt.5]) -> tilt 0.
    assert tilt_from_vertical_deg(0, -np.sqrt(.5), 0, np.sqrt(.5)) == pytest.approx(0, abs=1e-6)
    # 항등 쿼터니언(기수 +x, 세계수평) -> tilt 90.
    assert tilt_from_vertical_deg(0, 0, 0, 1) == pytest.approx(90, abs=1e-6)


def test_plot_run_writes_a_png_from_a_synthetic_csv(tmp_path):
    n = 20
    t = np.linspace(0, 2, n)
    df = pd.DataFrame({
        "t_s": t, "pos_x_m": t*8, "pos_y_m": 0.0, "pos_z_m": 20.0,
        "vx_mps": 8.0, "vy_mps": 0.0, "vz_mps": 0.0,
        "qx": 0.0, "qy": -np.sqrt(.5), "qz": 0.0, "qw": np.sqrt(.5),
        "wx_rads": 0.0, "wy_rads": 0.0, "wz_rads": 0.0,
        "n1_rads": 500.0, "n2_rads": 500.0, "n3_rads": 500.0, "n4_rads": 500.0,
        "u1_rads": 500.0, "u2_rads": 500.0, "u3_rads": 500.0, "u4_rads": 500.0,
        "ref_vx_mps": 8.0, "ref_vy_mps": 0.0, "ref_vz_mps": 0.0, "ref_z_m": 20.0,
        "soc": np.linspace(1, .9, n),
        "voltage_V": 24.9, "bus_current_A": 18.0, "power_W": 450.0,
        "thrust1_N": 4.2, "thrust2_N": 4.2, "thrust3_N": 4.2, "thrust4_N": 4.2,
    })
    csv_path = tmp_path/"synthetic.csv"
    df.to_csv(csv_path, index=False)
    out_path = plot_run(str(csv_path))
    assert out_path.endswith(".png")
    from pathlib import Path
    assert Path(out_path).stat().st_size > 1000


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
