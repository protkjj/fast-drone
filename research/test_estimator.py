import numpy as np
from research.eskf import build, SENSORS
from research.model import initial_state, profile, rotation


def initial():
    p = profile()
    x = initial_state(p)
    nominal = np.r_[x[:10], np.zeros(6)]
    covariance = np.eye(15)*.01
    imu = np.r_[np.array(rotation(x[6:10])).T @ [0,0,p["g"]], [0,0,0]]
    return p, nominal, covariance, imu


def test_eskf_hover_and_positive_covariance():
    p, state, cov, imu = initial()
    functions = build(p["g"])
    expected = state.copy()
    for i in range(300):
        state, flat = functions["predict"](state, cov.ravel(order="F"), imu)
        state = np.asarray(state).ravel()
        cov = np.asarray(flat).reshape((15,15), order="F")
        if (i+1)%100 == 0:
            state, flat, _ = functions["gps"](state,cov.ravel(order="F"),expected[:6])
            state = np.asarray(state).ravel()
            cov = np.asarray(flat).reshape((15,15), order="F")
    np.testing.assert_allclose(state, expected, atol=1e-10)
    np.testing.assert_allclose(cov, cov.T, atol=1e-12)
    assert np.linalg.eigvalsh(cov).min() > 0


def test_per_sample_imu_noise_has_dt_squared_covariance():
    p, state, cov, imu = initial()
    functions = build(p["g"])
    _, flat = functions["predict"](state,np.zeros(225),imu)
    cov = np.asarray(flat).reshape((15,15),order="F")
    np.testing.assert_allclose(np.diag(cov)[3:6],SENSORS["accel_std_mps2"]**2*.001**2,rtol=1e-10)
    np.testing.assert_allclose(np.diag(cov)[6:9],SENSORS["gyro_std_rad_s"]**2*.001**2,rtol=1e-10)


def test_gps_injection_preserves_unit_quaternion_and_spd():
    p,state,cov,imu=initial()
    functions=build(p["g"])
    for _ in range(100):
        state,flat=functions["predict"](state,cov.ravel(order="F"),imu)
        cov=np.asarray(flat).reshape((15,15),order="F")
    state,flat,nis=functions["gps"](state,cov.ravel(order="F"),[1,-2,20.5,.3,-.2,.1])
    state=np.asarray(state).ravel();cov=np.asarray(flat).reshape((15,15),order="F")
    assert np.isclose(np.linalg.norm(state[6:10]),1)
    assert np.linalg.eigvalsh(cov).min()>0
    assert float(nis)>0
