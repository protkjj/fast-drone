// 겹 1 — 수식 이식 검증 (DESIGN.md 4장)
//
// Gazebo 없이 돈다. macOS 에서도 빌드된다. 그게 이 테스트의 존재 이유다:
// "수식을 제대로 옮겼는가"와 "프레임 변환이 맞는가"를 **분리해서** 잡기 위해서다.
// 하나로 묶으면 실패했을 때 원인이 어느 쪽인지 못 가린다.
//
// 빌드 (gz 불필요):
//   c++ -std=c++17 -O2 -I gz_aero/include gz_aero/test/test_aero_core.cc \
//       -o /tmp/test_aero_core
//   /tmp/test_aero_core gz_aero/data/aero_sized.csv \
//                       gz_aero/test/aero_reference_sized.csv
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "fast_drone/aero_core.hpp"
#include "fast_drone/aero_table.hpp"
#include "fast_drone/frame.hpp"

using namespace fast_drone;

// ── 아주 작은 테스트 하네스 ────────────────────────────────────────────
static int g_fail = 0;
static int g_pass = 0;

static void Check(bool cond, const std::string &what) {
  if (cond) { ++g_pass; }
  else { ++g_fail; std::printf("  [FAIL] %s\n", what.c_str()); }
}

static void CheckClose(double got, double want, double atol, double rtol,
                       const std::string &what) {
  const double tol = atol + rtol * std::fabs(want);
  const bool ok = std::fabs(got - want) <= tol;
  if (ok) { ++g_pass; }
  else {
    ++g_fail;
    std::printf("  [FAIL] %s\n         got %.17g  want %.17g  diff %.3e  tol %.3e\n",
                what.c_str(), got, want, got - want, tol);
  }
}

// ══════════════════════════════════════════════════════════════════════
// 1. 좌표 변환 — 손으로 계산한 값과 대조 (DESIGN.md 2장)
// ══════════════════════════════════════════════════════════════════════
static void TestFrame() {
  std::printf("[1] 좌표 변환 (L <-> B)\n");
  std::string err;

  // (a) 기본값: 기수 = 링크 +x. FLU -> FRD 는 x축 180도 회전 = diag(1,-1,-1)
  {
    FrameLB f;
    Check(BuildFrameLB({{1, 0, 0}}, {{0, -1, 0}}, &f, &err), "기본 프레임 생성: " + err);
    Check(f.down == Vec3{{0, 0, -1}}, "기본 down = (0,0,-1)");
    const Vec3 b = f.ToBody({{1, 2, 3}});
    Check(b == Vec3{{1, -2, -3}}, "ToBody(1,2,3) = (1,-2,-3)");
    const Vec3 l = f.ToLink({{1, 2, 3}});
    Check(l == Vec3{{1, -2, -3}}, "ToLink(1,2,3) = (1,-2,-3)");
    CheckClose(f.Determinant(), 1.0, 1e-12, 0, "기본 det = +1");
  }

  // (b) 테일시터 후보: 기수 = 링크 +z (추력축을 장축에 맞출 때)
  //     down = nose x right = (0,0,1) x (0,-1,0) = (1,0,0)
  {
    FrameLB f;
    Check(BuildFrameLB({{0, 0, 1}}, {{0, -1, 0}}, &f, &err), "테일시터 프레임 생성: " + err);
    Check(f.down == Vec3{{1, 0, 0}}, "테일시터 down = (1,0,0)");
    // v_L=(1,2,3) -> v_B = (dot(nose,v), dot(right,v), dot(down,v)) = (3,-2,1)
    Check(f.ToBody({{1, 2, 3}}) == Vec3{{3, -2, 1}}, "테일시터 ToBody(1,2,3) = (3,-2,1)");
    CheckClose(f.Determinant(), 1.0, 1e-12, 0, "테일시터 det = +1");
  }

  // (c) 왕복: ToLink(ToBody(v)) == v  (어떤 프레임이든)
  {
    FrameLB f;
    BuildFrameLB({{0, 0, 1}}, {{0, -1, 0}}, &f, &err);
    const Vec3 v{{1.37, -2.91, 0.44}};
    const Vec3 rt = f.ToLink(f.ToBody(v));
    for (int i = 0; i < 3; ++i)
      CheckClose(rt[i], v[i], 1e-15, 1e-15, "왕복 성분 " + std::to_string(i));
  }

  // (d) 거부해야 하는 입력
  {
    FrameLB f;
    Check(!BuildFrameLB({{1, 0, 0}}, {{1, 0, 0}}, &f, &err),
          "직교하지 않는 nose/right 를 거부해야 한다");
    Check(!BuildFrameLB({{0, 0, 0}}, {{0, -1, 0}}, &f, &err),
          "영벡터 nose 를 거부해야 한다");
    Check(!BuildFrameLB({{1, 0, 0}}, {{0, std::nan(""), 0}}, &f, &err),
          "NaN 이 든 right 를 거부해야 한다");
  }

  // (e) 정규화: 크기가 1 이 아니어도 받아서 정규화한다
  {
    FrameLB f;
    Check(BuildFrameLB({{5, 0, 0}}, {{0, -3, 0}}, &f, &err), "비정규 벡터 정규화: " + err);
    Check(f.nose == Vec3{{1, 0, 0}} && f.right == Vec3{{0, -1, 0}}, "정규화 결과");
  }

  // (f) det 검사가 실제로 작동하는지 — down 을 손으로 망가뜨려 본다
  //     (BuildFrameLB 는 down 을 항상 유도하므로 정상 경로로는 여기 못 온다.
  //      이건 "수치 오염이 생겼을 때 걸리는가"를 보는 것이다)
  {
    FrameLB bad;
    bad.nose = {{1, 0, 0}}; bad.right = {{0, -1, 0}}; bad.down = {{0, 0, 1}};
    Check(bad.Determinant() < 0, "왼손 좌표계면 det 가 음수여야 한다");
  }
  std::printf("\n");
}

// ══════════════════════════════════════════════════════════════════════
// 2. 기준값 CSV 읽기
// ══════════════════════════════════════════════════════════════════════
struct RefRow {
  std::string name;
  int on_grid = 1;
  Vec3 v_b, w_b, F, M;
};

static bool LoadReference(const std::string &path, std::vector<RefRow> *rows,
                          BodyParams *bp, std::string *err) {
  std::ifstream f(path);
  if (!f.is_open()) { *err = "기준값 CSV 를 열 수 없습니다: " + path; return false; }
  std::map<std::string, size_t> col;
  bool header = false;
  size_t ln = 0;
  std::string line;
  while (std::getline(f, line)) {
    ++ln;
    if (ln == 1) detail::StripBom(&line);
    detail::TrimInPlace(&line);
    if (line.empty()) continue;
    if (line[0] == '#') {
      const size_t c = line.find(':');
      if (c == std::string::npos) continue;
      std::string k = line.substr(1, c - 1), v = line.substr(c + 1);
      detail::TrimInPlace(&k); detail::TrimInPlace(&v);
      double d;
      if (k == "S_ref"   && detail::ParseDouble(v, &d)) bp->s_ref = d;
      if (k == "d_ref"   && detail::ParseDouble(v, &d)) bp->d_ref = d;
      if (k == "rho_ref" && detail::ParseDouble(v, &d)) bp->rho   = d;
      continue;
    }
    auto cells = detail::SplitCsv(line);
    if (!header) {
      for (size_t i = 0; i < cells.size(); ++i) col[cells[i]] = i;
      header = true;
      continue;
    }
    RefRow r;
    auto num = [&](const char *n) {
      double d = 0.0;
      detail::ParseDouble(cells[col[n]], &d);
      return d;
    };
    r.name = cells[col["case"]];
    r.on_grid = static_cast<int>(num("grid"));
    r.v_b = {{num("u"), num("v"), num("w")}};
    r.w_b = {{num("p"), num("q"), num("r")}};
    r.F   = {{num("Fx"), num("Fy"), num("Fz")}};
    r.M   = {{num("Mx"), num("My"), num("Mz")}};
    rows->push_back(r);
  }
  return !rows->empty();
}

// ══════════════════════════════════════════════════════════════════════
int main(int argc, char **argv) {
  if (argc < 3) {
    std::printf("사용법: %s <표 CSV> <기준값 CSV> [보간스윕 CSV]\n", argv[0]);
    return 2;
  }
  const std::string table_path = argv[1];
  const std::string ref_path = argv[2];

  TestFrame();

  // ── 표 로드 + 겹0 상식 검사 ──────────────────────────────────────────
  std::printf("[2] 표 로드 + 상식 검사\n");
  AeroTable table;
  std::string err;
  if (!AeroTable::Load(table_path, &table, &err)) {
    std::printf("  [FAIL] 표 로드 실패: %s\n", err.c_str());
    return 1;
  }
  std::printf("  스키마 %s | 격자 V %zu x alpha %zu | S_ref %.8g d_ref %.8g\n",
              table.schema().c_str(), table.n_v(), table.n_alpha(),
              table.s_ref(), table.d_ref());
  std::vector<std::string> warn;
  Check(table.SanityCheck(&warn, &err), "겹0 상식 검사: " + err);
  for (const auto &w : warn) std::printf("  [경고] %s\n", w.c_str());
  std::printf("\n");

  // ── 기준값 로드 ──────────────────────────────────────────────────────
  std::vector<RefRow> rows;
  BodyParams bp;
  if (!LoadReference(ref_path, &rows, &bp, &err)) {
    std::printf("  [FAIL] %s\n", err.c_str());
    return 1;
  }
  Check(CheckRefConsistency(table, bp, &err), "표 <-> 기준값 S_ref/d_ref 정합: " + err);

  // ── 겹1 test A : 격자점 위 = 보간오차 0 = 순수 "수식 이식" 검증 ────────
  // CSV 계수를 %.17g (double 무손실) 로 적으므로 오차 바닥이 반올림(~1e-15)이다.
  // 1e-12 는 그보다 세 자릿수 위 — 진짜 이식 오류는 무조건 걸리고
  // 부동소수 반올림에는 안 걸리는 자리다.
  // 허용오차의 바닥은 **정체가 밝혀진 것**이다. 모르는 채로 느슨하게 잡은
  // 값이 아니다.
  //
  //  파이썬 원본은 V_cf = sqrt(v^2+w^2 + EPS) 로 EPS 를 더해 0 나눗셈을 피한다.
  //  우리는 각도 계산에서만 그 EPS 를 뺐다 (aero_core.hpp 주석 참고 — 안 빼면
  //  alpha=0 에서 표를 0 행이 아니라 그 옆을 섞어 읽는다). 그 차이가 크로스플로
  //  항에 C_dc*EPS/(2*V_cf) 만큼 남는다:
  //       V = 5 m/s  ->  ~2e-10        V = 1 m/s  ->  ~5e-9
  //  q_bar 와 감쇠가 쓰는 V_sq 는 원본의 EPS 를 그대로 두었으므로 그쪽은 0 이다.
  //
  //  atol 1e-11 [N] 은 V=0 케이스용이다. 그 상태에서 원본은 0.5*rho*EPS 로
  //  1e-11 N 짜리 힘을 내고, 우리는 표를 조회해 미세하게 다른 값을 낸다.
  //  힘 자체가 1e-11 N 이라 물리적으로 무의미하다.
  //
  // 1e-9 는 여전히 극도로 빡빡하다. 부호 반전, 축 뒤바뀜, 계수 오배치 같은
  // **진짜 이식 오류는 전부 O(1)** 이라 무조건 걸린다.
  const double kAtol = 1e-11;
  const double kRtol = 1e-9;

  std::printf("[3] test A — 격자점 위, 파이썬 _body_aerodynamics 대조 "
              "(허용 rel 1e-9 / abs 1e-11)\n");
  std::printf("  %-14s %11s %11s %11s %11s %11s %11s\n",
              "case", "dFx", "dFy", "dFz", "dMx", "dMy", "dMz");
  for (const auto &r : rows) {
    const Wrench got = ComputeBodyWrench(table, bp, r.v_b, r.w_b);
    const char *nm[6] = {"Fx", "Fy", "Fz", "Mx", "My", "Mz"};
    double g[6] = {got.force[0], got.force[1], got.force[2],
                   got.moment[0], got.moment[1], got.moment[2]};
    double e[6] = {r.F[0], r.F[1], r.F[2], r.M[0], r.M[1], r.M[2]};
    double rel[6];
    for (int i = 0; i < 6; ++i) {
      const double den = std::fabs(e[i]) > 1e-9 ? std::fabs(e[i]) : 1.0;
      rel[i] = std::fabs(g[i] - e[i]) / den;
      CheckClose(g[i], e[i], kAtol, kRtol, r.name + " / " + nm[i]);
    }
    std::printf("  %-14s %11.2e %11.2e %11.2e %11.2e %11.2e %11.2e\n",
                r.name.c_str(), rel[0], rel[1], rel[2], rel[3], rel[4], rel[5]);
  }
  std::printf("\n");

  // ── 겹1 test B : 격자 **모든 칸의 중점** = 선형보간 오차 최대점 ─────────
  // 점 몇 개만 찍어보고 "1% 안에 든다"고 말하지 않는다. 칸 전부를 훑는다.
  if (argc >= 4) {
    std::printf("[3b] test B — 격자칸 중점 전수 스윕 (보간 품질 측정)\n");
    std::vector<RefRow> sweep;
    BodyParams bp2;
    if (!LoadReference(argv[3], &sweep, &bp2, &err)) {
      std::printf("  [FAIL] %s\n", err.c_str());
      return 1;
    }
    // 지표는 **계수 오차** dC = |dF| / (q_bar * S_ref) 로 잰다.
    //
    // 왜 성분별 상대오차를 안 쓰나: C_N 은 alpha ~ 129도 근처에서 0 을 지난다.
    // 거기서는 |F| 자체가 0 에 가까워 분모가 죽고, 지표가 3% 로 튀지만 실제
    // 오차는 0.0001 N 이다. 물리적으로 아무 의미 없는 숫자에 테스트가 걸린다.
    //
    // 계수 기준이면 "표를 얼마나 촘촘히 떴는가" 를 속도와 무관하게 직접 잰다.
    // C_N 최대값이 11.5 이므로 dC_N = 0.01 은 최댓값의 0.09% 다.
    auto norm3 = [](double a, double b, double c) {
      return std::sqrt(a*a + b*b + c*c);
    };
    double wCA = 0.0, wCN = 0.0, wCm = 0.0, wFabs = 0.0, wMabs = 0.0;
    std::string cCA, cCN, cCm, cFabs, cMabs;
    size_t skipped = 0;
    for (const auto &r : sweep) {
      const Wrench got = ComputeBodyWrench(table, bp2, r.v_b, r.w_b);
      const double Vsq = r.v_b[0]*r.v_b[0] + r.v_b[1]*r.v_b[1] +
                         r.v_b[2]*r.v_b[2] + kEps;
      const double qS = 0.5 * bp2.rho * Vsq * bp2.s_ref;
      const double dF = norm3(got.force[0]-r.F[0], got.force[1]-r.F[1],
                              got.force[2]-r.F[2]);
      const double dM = norm3(got.moment[0]-r.M[0], got.moment[1]-r.M[1],
                              got.moment[2]-r.M[2]);
      if (dF > wFabs) { wFabs = dF; cFabs = r.name; }
      if (dM > wMabs) { wMabs = dM; cMabs = r.name; }
      if (qS < 1e-12) { ++skipped; continue; }
      const double dCA = std::fabs(got.force[0]-r.F[0]) / qS;
      const double dCN = norm3(0.0, got.force[1]-r.F[1], got.force[2]-r.F[2]) / qS;
      const double dCm = dM / (qS * bp2.d_ref);
      if (dCA > wCA) { wCA = dCA; cCA = r.name; }
      if (dCN > wCN) { wCN = dCN; cCN = r.name; }
      if (dCm > wCm) { wCm = dCm; cCm = r.name; }
    }

    // 판정은 **힘 오차**로 한다. 계수 오차는 진단이다.
    //
    // 계수로 판정하면 저속에서 걸린다: V=3.5 m/s 에서 dC_A=1.3e-2 는 힘으로
    // 6e-4 N 이라 비행에 아무 영향이 없는데 지표만 나쁘다. 반대로 고속에선
    // dC_N=3e-4 가 0.02 N 을 만든다. 물리적으로 의미 있는 건 힘 쪽이다.
    //
    // 허용치는 표가 들고 있다 (# F_tol_N). 표를 만든 적응격자가 그 값을 목표로
    // 쪼갰으므로, 표와 테스트가 같은 숫자를 본다.
    //
    // 2배를 주는 이유: 적응격자는 V 축과 alpha 축을 **따로** 재서 각각
    // F_tol 이하로 맞춘다. 실제 2차원 보간에서는 두 오차가 겹칠 수 있다.
    const double ftol = (table.f_tol_n() > 0.0) ? table.f_tol_n() : 0.05;
    const double kAxes = 2.0;
    const double f_limit = kAxes * ftol;
    const double m_limit = kAxes * ftol * bp2.d_ref;

    std::printf("  스윕 점 %zu 개  (이름 sNN_MMM = V칸 NN, alpha칸 MMM 의 중점)"
                "%s\n", sweep.size(),
                skipped ? ("  [q_bar~0 로 건너뜀 " + std::to_string(skipped) +
                           "개]").c_str() : "");
    std::printf("  ▶ 힘   최대 |dF| = %.3e N     (%s)   [허용 %.3g N]\n",
                wFabs, cFabs.c_str(), f_limit);
    std::printf("  ▶ 모멘트 최대 |dM| = %.3e N m   (%s)   [허용 %.3g N m]\n",
                wMabs, cMabs.c_str(), m_limit);
    std::printf("    (진단) 계수오차  dC_A %.3e (%s)  dC_N %.3e (%s)  dC_m %.3e (%s)\n",
                wCA, cCA.c_str(), wCN, cCN.c_str(), wCm, cCm.c_str());
    Check(wFabs < f_limit, "보간이 만드는 힘 오차가 허용치 이내");
    Check(wMabs < m_limit, "보간이 만드는 모멘트 오차가 허용치 이내");
    std::printf("\n");
  }

  // ── 축대칭 확인: 크로스플로를 +y 로 준 것과 +z 로 준 것의 크기가 같아야 ──
  std::printf("[4] 축대칭 확인 (swap_yz 쌍)\n");
  {
    const RefRow *a = nullptr, *b = nullptr;
    for (const auto &r : rows) {
      if (r.name == "swap_yz_a") a = &r;
      if (r.name == "swap_yz_b") b = &r;
    }
    if (a && b) {
      const Wrench wa = ComputeBodyWrench(table, bp, a->v_b, a->w_b);
      const Wrench wb = ComputeBodyWrench(table, bp, b->v_b, b->w_b);
      auto mag = [](const Vec3 &v) { return std::sqrt(Dot(v, v)); };
      CheckClose(mag(wa.force), mag(wb.force), 1e-12, 1e-12, "|F| 가 같아야 한다");
      CheckClose(mag(wa.moment), mag(wb.moment), 1e-12, 1e-12, "|M| 가 같아야 한다");
      CheckClose(wa.force[0], wb.force[0], 1e-12, 1e-12, "축력이 같아야 한다");
      std::printf("  |F| %.6f vs %.6f    |M| %.6f vs %.6f\n",
                  mag(wa.force), mag(wb.force), mag(wa.moment), mag(wb.moment));
    } else {
      Check(false, "swap_yz_a / swap_yz_b 케이스를 못 찾음");
    }
  }
  std::printf("\n");

  // ── NaN 방어 ─────────────────────────────────────────────────────────
  std::printf("[5] NaN 방어\n");
  {
    const Wrench w = ComputeBodyWrench(table, bp, {{std::nan(""), 0, 0}}, {{0, 0, 0}});
    Check(w.force[0] == 0 && w.force[1] == 0 && w.force[2] == 0 &&
          w.moment[0] == 0 && w.moment[1] == 0 && w.moment[2] == 0,
          "NaN 입력이면 0 wrench 를 내야 한다");
  }
  std::printf("\n");

  std::printf("═══ 통과 %d / 실패 %d ═══\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
