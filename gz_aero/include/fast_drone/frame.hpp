// 좌표계 변환 — 링크(L, Gazebo FLU) <-> 동체(B, 미사일 FRD)
//
// DESIGN.md 2장이 이 파일의 계약서다. 여기가 틀리면 시뮬은 멀쩡히 돌고 결과만
// 조용히 틀린다. 그래서 이 파일은 짧고, 검사가 많고, 테스트가 붙는다.
//
// ── C++ 처음 보는 사람을 위한 메모 ─────────────────────────────────────────
//  * `#pragma once` : 이 파일이 여러 곳에서 include 돼도 한 번만 펼쳐지게 한다.
//                     (파이썬 import 는 알아서 해주지만 C++ 은 말 그대로 "복붙"이라
//                      중복 정의 에러가 난다)
//  * "헤더 온리"    : 선언과 구현을 .hpp 하나에 다 넣었다. 함수가 짧고, 이러면
//                     따로 .cc 를 컴파일해 링크할 필요가 없어 테스트 빌드가 쉽다.
//                     그래서 함수 앞에 `inline` 을 붙인다 (중복 정의 방지).
//  * `const T&`     : "읽기만 하고 복사도 안 한다". 파이썬에는 없는 개념인데,
//                     C++ 은 기본이 값 복사라 큰 객체는 이렇게 참조로 넘긴다.
//  * `std::string*` : 에러 메시지를 돌려주는 출력 인자. 예외(exception)를 안 쓰는 건
//                     Gazebo 플러그인 안에서 예외가 터지면 시뮬 전체가 죽기 때문이다.
// ──────────────────────────────────────────────────────────────────────────
#pragma once

#include <array>
#include <cmath>
#include <string>

namespace fast_drone {

// 3차원 벡터. std::array 는 크기가 고정된 배열이라 힙 할당이 없다(빠르다).
using Vec3 = std::array<double, 3>;

inline double Dot(const Vec3 &a, const Vec3 &b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline Vec3 Cross(const Vec3 &a, const Vec3 &b) {
  return {a[1] * b[2] - a[2] * b[1],
          a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

inline double Norm(const Vec3 &a) { return std::sqrt(Dot(a, a)); }

/// 링크 프레임(L) 안에서 동체 프레임(B)의 세 축을 들고 있는다.
///
/// R_LB = [nose | right | down]  (열벡터 3개)
///   B->L :  v_L = R_LB * v_B
///   L->B :  v_B = R_LB^T * v_L
///
/// 행렬을 double[3][3] 로 안 만들고 축 벡터 3개로 둔 이유: 곱셈이 내적/선형결합
/// 으로 그대로 읽히기 때문이다. 인덱스를 헷갈릴 자리가 줄어든다.
struct FrameLB {
  Vec3 nose{{1.0, 0.0, 0.0}};    ///< B 의 +x (기수)   를 L 성분으로
  Vec3 right{{0.0, -1.0, 0.0}};  ///< B 의 +y (우현)   를 L 성분으로
  Vec3 down{{0.0, 0.0, -1.0}};   ///< B 의 +z (아래)   = nose x right (유도값)

  /// L -> B : 링크 좌표 벡터를 동체 좌표로.
  ///   v_B = R^T v_L 인데, R 의 열이 축이므로 R^T 의 행이 축이다.
  ///   따라서 각 성분이 그냥 "축과의 내적"이 된다.
  Vec3 ToBody(const Vec3 &v_L) const {
    return {Dot(nose, v_L), Dot(right, v_L), Dot(down, v_L)};
  }

  /// B -> L : 동체 좌표 벡터를 링크 좌표로.
  ///   v_L = R v_B = v_B[0]*nose + v_B[1]*right + v_B[2]*down
  Vec3 ToLink(const Vec3 &v_B) const {
    return {nose[0] * v_B[0] + right[0] * v_B[1] + down[0] * v_B[2],
            nose[1] * v_B[0] + right[1] * v_B[1] + down[1] * v_B[2],
            nose[2] * v_B[0] + right[2] * v_B[1] + down[2] * v_B[2]};
  }

  /// 힘과 모멘트에 **같은** 변환을 써도 되는지 확인한다.
  ///
  /// 모멘트는 유사벡터(pseudovector)라 반사(det = -1)에서는 벡터와 다르게
  /// 변환된다. det = +1 인 진짜 회전이면 둘이 같다. 그래서 여기서 막는다.
  double Determinant() const { return Dot(nose, Cross(right, down)); }
};

/// SDF 가 준 nose/right 두 벡터로 프레임을 만든다. 실패하면 false + 에러 메시지.
///
/// 세 번째 축(down)은 **사람이 적게 하지 않고 여기서 유도**한다. 손으로 적으면
/// 왼손좌표계를 만들어 놓고 못 알아채기 때문이다.
inline bool BuildFrameLB(Vec3 nose, Vec3 right, FrameLB *out, std::string *err) {
  auto fail = [err](const std::string &m) {
    if (err) *err = m;
    return false;
  };
  if (!out) return fail("BuildFrameLB: out 이 nullptr");

  const double n_nose = Norm(nose);
  const double n_right = Norm(right);
  if (!(n_nose > 1e-9) || !std::isfinite(n_nose))
    return fail("nose 벡터의 크기가 0 이거나 NaN 입니다");
  if (!(n_right > 1e-9) || !std::isfinite(n_right))
    return fail("right 벡터의 크기가 0 이거나 NaN 입니다");

  for (int i = 0; i < 3; ++i) {
    nose[i] /= n_nose;
    right[i] /= n_right;
  }

  // 직교성. 1e-6 은 "손으로 적은 값이 조금 틀어진 것"은 봐주고
  // "축을 잘못 골랐다"는 확실히 잡는 경계다 (0.06도 이내).
  const double cosang = Dot(nose, right);
  if (std::fabs(cosang) > 1e-6) {
    return fail("nose 와 right 가 직교하지 않습니다 (dot = " +
                std::to_string(cosang) + "). FRD 에서 기수와 우현은 90도여야 합니다");
  }

  out->nose = nose;
  out->right = right;
  out->down = Cross(nose, right);   // FRD: 앞 x 우 = 아래

  const double det = out->Determinant();
  if (!(det > 0.999) || !(det < 1.001)) {
    return fail("좌표계 행렬식이 " + std::to_string(det) +
                " 입니다 (+1 이어야 함). 오른손 좌표계가 아니면 모멘트 부호가 "
                "조용히 뒤집힙니다");
  }
  if (err) err->clear();
  return true;
}

}  // namespace fast_drone
