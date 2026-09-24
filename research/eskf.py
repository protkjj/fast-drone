"""Serializable 15D local-error ESKF, IMU 1 kHz / GPS 10 Hz.

Uses per-sample sensor standard deviations, not ambiguously named noise density.
GPS correction uses Joseph form AND a local attitude-error covariance reset
(Sola 2017, section 6.3). No accelerometer-as-gravity pseudo-observation.
"""
import casadi as ca
from research.model import rotation, DT

SENSORS = {"accel_std_mps2": .02, "gyro_std_rad_s": .001,
           "accel_bias_std_mps2": .05, "gyro_bias_std_rad_s": .005,
           "accel_bias_rw_mps2_sqrt_s": 1e-4, "gyro_bias_rw_rad_s_sqrt_s": 1e-5,
           "gps_position_std_m": 1.5, "gps_velocity_std_mps": .5,
           "gps_period_ticks": 100, "gps_latency_ticks": 20, "rpm_std_rad_s": 1,
           "note": "Assumed noise and 20 ms GPS delay; not hardware specifications"}


def skew(v):
    return ca.vertcat(ca.horzcat(0, -v[2], v[1]), ca.horzcat(v[2], 0, -v[0]), ca.horzcat(-v[1], v[0], 0))


def multiply(a, b):
    return ca.vertcat(a[3]*b[:3]+b[3]*a[:3]+ca.cross(a[:3], b[:3]), a[3]*b[3]-ca.dot(a[:3], b[:3]))


def exp_quat(theta):
    angle = ca.sqrt(ca.sumsqr(theta)+1e-24)
    return ca.vertcat(theta*ca.sin(angle/2)/angle, ca.cos(angle/2))


def build(g):
    s = ca.SX.sym("nominal", 16)  # p,v,q,ba,bg
    cov = ca.SX.sym("covariance", 15, 15)
    imu = ca.SX.sym("imu", 6)
    acc, omega = imu[:3]-s[10:13], imu[3:]-s[13:16]
    r = rotation(s[6:10])
    inertial = r @ acc + ca.DM([0,0,-g])
    q = multiply(s[6:10], exp_quat(omega*DT))
    nxt = ca.vertcat(s[:3]+s[3:6]*DT+.5*inertial*DT**2,
                    s[3:6]+inertial*DT, q/ca.norm_2(q), s[10:])
    transition = ca.SX.eye(15)
    transition[:3, 3:6] = ca.DM.eye(3)*DT
    transition[:3, 6:9] = -.5*r @ skew(acc)*DT**2
    transition[:3, 9:12] = -.5*r*DT**2
    transition[3:6, 6:9] = -r @ skew(acc)*DT
    transition[3:6, 9:12] = -r*DT
    transition[6:9, 6:9] = ca.DM.eye(3)-skew(omega)*DT
    transition[6:9, 12:15] = -ca.DM.eye(3)*DT
    noise = ca.SX.zeros(15, 12)
    noise[:3, :3] = .5*r*DT**2
    noise[3:6, :3] = r*DT
    noise[6:9, 3:6] = ca.DM.eye(3)*DT
    noise[9:12, 6:9] = ca.DM.eye(3)*DT**.5
    noise[12:15, 9:12] = ca.DM.eye(3)*DT**.5
    sigmas = ([SENSORS["accel_std_mps2"]]*3+[SENSORS["gyro_std_rad_s"]]*3
              +[SENSORS["accel_bias_rw_mps2_sqrt_s"]]*3+[SENSORS["gyro_bias_rw_rad_s_sqrt_s"]]*3)
    prediction = transition @ cov @ transition.T + noise @ ca.diag(ca.DM(sigmas)**2) @ noise.T
    predict = ca.Function("eskf_predict", [s, ca.vec(cov), imu], [nxt, ca.vec((prediction+prediction.T)/2)])

    gps = ca.SX.sym("gps", 6)
    h = ca.DM.zeros(6, 15)
    h[:6, :6] = ca.DM.eye(6)
    measurement_cov = ca.diag(ca.DM([SENSORS["gps_position_std_m"]**2]*3+[SENSORS["gps_velocity_std_mps"]**2]*3))
    innovation = gps-s[:6]
    innovation_cov = h @ cov @ h.T + measurement_cov
    gain = ca.solve(innovation_cov, h @ cov).T
    delta = gain @ innovation
    corrected_q = multiply(s[6:10], exp_quat(delta[6:9]))
    corrected = ca.vertcat(s[:3]+delta[:3], s[3:6]+delta[3:6], corrected_q/ca.norm_2(corrected_q),
                          s[10:13]+delta[9:12], s[13:16]+delta[12:15])
    ikh = ca.DM.eye(15)-gain @ h
    posterior = ikh @ cov @ ikh.T + gain @ measurement_cov @ gain.T
    reset = ca.SX.eye(15)
    reset[6:9, 6:9] = ca.DM.eye(3)-.5*skew(delta[6:9])
    posterior = reset @ posterior @ reset.T
    update = ca.Function("eskf_gps", [s, ca.vec(cov), gps],
                         [corrected, ca.vec((posterior+posterior.T)/2), ca.dot(innovation, ca.solve(innovation_cov, innovation))])
    return {"predict": predict, "gps": update}
