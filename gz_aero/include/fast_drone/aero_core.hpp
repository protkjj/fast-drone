// 공력 코어 — 동체(B) 좌표에서 힘·모멘트를 계산한다.
//
// ★ 이 파일은 Gazebo 헤더를 하나도 include 하지 않는다. 일부러 그렇게 했다.
//   그래야 gz 없이도 빌드·실행되고, macOS 에서 파이썬 기준값과 대조할 수 있다.
//   (DESIGN.md 4장 "겹 1")
//
// 아래 수식은 control/dynamics.py 의 `_body_aerodynamics()` 를 한 줄씩 옮긴 것이다.
// 대응되는 파이썬 줄을 주석에 같이 적어 뒀으니 나란히 놓고 읽으면 된다.
#pragma once

#include <cmath>
#include <string>

#include "fast_drone/aero_table.hpp"
#include "fast_drone/frame.hpp"

namespace fast_drone {

/// 파이썬 원본(control/dynamics.py:23)과 **같은 값**을 쓴다.
///
/// 왜 굳이 이 수치 꼼수까지 옮기나: 이게 있어야 V_cf 가 절대 0 이 안 되어
/// 0 으로 나누는 자리가 사라지고(원본이 "분모 소거"라고 부른 것), 동시에
/// 파이썬 기준값과 **1e-9 수준까지** 맞출 수 있다. 값이 다르면 V=0 근처에서
/// 미세하게 갈라져 "이 차이가 진짜인가 엡실론인가"를 매번 따져야 한다.
inline constexpr double kEps = 1e-8;

/// 힘 계산에 필요한 기체·환경 상수. 표에는 없고 SDF 가 준다.
struct BodyParams {
  double s_ref = 0.0;   ///< [m^2] 기준면적
  double d_ref = 0.0;   ///< [m]   기준길이
  double rho = 1.225;   ///< [kg/m^3] 실제 대기밀도 (표의 rho_ref 와는 별개)
};

/// 동체(B) 좌표의 힘[N]과 모멘트[N·m]. 모멘트는 CG 기준.
struct Wrench {
  Vec3 force{{0.0, 0.0, 0.0}};
  Vec3 moment{{0.0, 0.0, 0.0}};
};

/// 진단용 중간값. 로그·디버그 토픽에 실어 보내면 원인 추적이 쉬워진다.
struct AeroDebug {
  double V = 0.0;        ///< 대기속도 크기 [m/s]
  double V_cf = 0.0;     ///< 크로스플로 속도 [m/s]
  double alpha = 0.0;    ///< 전 받음각 [rad]
  double q_bar = 0.0;    ///< 동압 [Pa]
  Coeffs c;              ///< 보간된 계수
};

/// 동체 좌표 대기속도 v_b 와 각속도 w_b 로부터 공력 wrench 를 낸다.
///
/// @param table  계수 표 (보간)
/// @param bp     S_ref, d_ref, rho
/// @param v_b    [m/s]   동체좌표 대기속도 (관성속도 - 바람, 을 B 로 회전한 것)
/// @param w_b    [rad/s] 동체좌표 각속도
/// @param dbg    nullptr 아니면 중간값을 채워 준다
///
/// 입력에 NaN/Inf 가 있으면 **0 wrench 를 돌려준다.** NaN 힘이 한 번 들어가면
/// 상태 전체가 NaN 이 되고, 그때부터는 무엇이 원인이었는지 알 수 없게 된다.
inline Wrench ComputeBodyWrench(const AeroTable &table, const BodyParams &bp,
                                const Vec3 &v_b, const Vec3 &w_b,
                                AeroDebug *dbg = nullptr) {
  Wrench out;  // 기본값 = 0

  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(v_b[i]) || !std::isfinite(w_b[i])) return out;
  }

  const double u = v_b[0], v = v_b[1], w = v_b[2];

  // ── dynamics.py:67-70 ─────────────────────────────────────────────────
  //   V_sq  = u**2 + v**2 + w**2 + EPS
  //   V     = ca.sqrt(V_sq)
  //   V_cf  = ca.sqrt(v**2 + w**2 + EPS)      <- ★ 여기만 원본과 다르게 간다
  //   q_bar = 0.5 * rho * V_sq
  //
  // ★ V_cf 에는 EPS 를 넣지 않는다. 원본이 EPS 를 넣은 이유는 0 으로 나누는
  //   것을 피하려는 것뿐인데, 그 대가로 **v=w=0 일 때 alpha 가 0 이 아니게**
  //   된다 (atan2(1e-4, u) = 1e-4/u). 원본은 계수를 직접 계산하므로 그래도
  //   괜찮았지만, 우리는 그 alpha 로 **표를 조회**하기 때문에 alpha=0 행이
  //   아니라 그 옆을 섞어 읽게 된다. C_A 가 alpha 에 의존하는 표에서는
  //   이게 1e-7 수준의 오차로 나타난다 (placeholder 소스에서 실측).
  //
  //   그래서 각도는 진짜 값으로 구하고, 0 나눗셈은 아래에서 명시적으로 막는다.
  //   V_sq / q_bar 는 원본의 EPS 를 그대로 둔다 — 그래야 V=0 에서도 원본과
  //   같은 값(1e-11 N 수준)이 나와 대조가 깔끔하다.
  const double vcf_sq = v * v + w * w;
  const double V_cf = std::sqrt(vcf_sq);
  const double V_sq = u * u + vcf_sq + kEps;
  const double V = std::sqrt(V_sq);
  const double q_bar = 0.5 * bp.rho * V_sq;

  // 전 받음각 [0, pi]. atan2 라 u < 0 (역류) 에서도 90~180도로 제대로 나온다.
  const double alpha = std::atan2(V_cf, u);

  const Coeffs c = table.Lookup(V, alpha);

  // ── 축력 : dynamics.py:78-79 ──────────────────────────────────────────
  //   C_A = C_A0 + C_Aa2 * (v**2 + w**2) / V_sq        <- 표가 이미 담고 있다
  //   Fx  = -q_bar * S * C_A
  const double Fx = -q_bar * bp.s_ref * c.C_A;

  // ── 법선력 : dynamics.py:73-75 ────────────────────────────────────────
  //   원본:  F_N_fac = 0.5*rho*S*(C_Na*u + C_dc*V_cf)
  //          Fy = -F_N_fac * v ,  Fz = -F_N_fac * w
  //   표판:  |F_N| = q_bar*S*C_N 이고 방향은 크로스플로 반대이므로
  //          Fy = -|F_N| * v/V_cf ,  Fz = -|F_N| * w/V_cf
  //   둘은 대수적으로 같다 (DESIGN.md 3장).
  const double F_N = q_bar * bp.s_ref * c.C_N;
  // V_cf = 0 이면 v = w = 0 이므로 법선력도 0 이다. 0/0 을 만들지 않고
  // 역수를 0 으로 둬서 결과가 **정확히** 0 이 되게 한다.
  const double inv_vcf = (V_cf > 0.0) ? (1.0 / V_cf) : 0.0;
  const double Fy = -F_N * v * inv_vcf;
  const double Fz = -F_N * w * inv_vcf;

  out.force = {Fx, Fy, Fz};

  // ── 정적 모멘트 : dynamics.py:85 ──────────────────────────────────────
  //   M_static = [0, -x_cp*Fz, +x_cp*Fy]      (= r_cp x F_N, r_cp = [x_cp,0,0])
  //   x_cp 가 음수(CG 뒤)이고 alpha 가 양수면 M_y 가 음수 = 기수 하강 = 복원.
  const double xcp = c.x_cp;
  double Mx = 0.0;
  double My = -xcp * Fz;
  double Mz = xcp * Fy;

  // ── 감쇠 모멘트 : dynamics.py:88-92 ───────────────────────────────────
  //   df = 0.25 * rho * V * S * d**2
  //   M_damp = df * [C_lp*p, C_mq*q, C_mq*r]
  //   V 의 1승인 것에 주의 (동압이 아니다). C_* 가 음수라 항상 omega 를 줄인다.
  const double df = 0.25 * bp.rho * V * bp.s_ref * bp.d_ref * bp.d_ref;
  Mx += df * c.C_lp * w_b[0];
  My += df * c.C_mq * w_b[1];
  Mz += df * c.C_mq * w_b[2];

  out.moment = {Mx, My, Mz};

  if (dbg) {
    dbg->V = V;
    dbg->V_cf = V_cf;
    dbg->alpha = alpha;
    dbg->q_bar = q_bar;
    dbg->c = c;
  }
  return out;
}

/// 표의 메타데이터와 SDF 가 준 값이 어긋나지 않는지 확인한다.
///
/// 이걸 안 하면: 표는 S_ref=0.00636 로 만들었는데 SDF 에 0.01767 이 남아 있어도
/// 시뮬은 멀쩡히 돌고 **모든 공력이 2.8배로 뻥튀기**된다. 조용히 틀리는 전형이다.
inline bool CheckRefConsistency(const AeroTable &table, const BodyParams &bp,
                                std::string *err) {
  auto rel = [](double a, double b) {
    const double d = std::fabs(a - b);
    const double s = std::fabs(b) > 1e-30 ? std::fabs(b) : 1.0;
    return d / s;
  };
  if (table.s_ref() > 0.0 && rel(bp.s_ref, table.s_ref()) > 1e-3) {
    if (err) *err = "S_ref 불일치: SDF " + std::to_string(bp.s_ref) +
                    " vs CSV " + std::to_string(table.s_ref()) +
                    " — 공력 전체가 그 비율만큼 조용히 스케일됩니다";
    return false;
  }
  if (table.d_ref() > 0.0 && rel(bp.d_ref, table.d_ref()) > 1e-3) {
    if (err) *err = "d_ref 불일치: SDF " + std::to_string(bp.d_ref) +
                    " vs CSV " + std::to_string(table.d_ref()) +
                    " — 감쇠 모멘트가 그 제곱만큼 틀어집니다";
    return false;
  }
  if (err) err->clear();
  return true;
}

}  // namespace fast_drone
