// 공력 계수 표 — CSV 로드 + 이중선형 보간 + 상식 검사
//
// DESIGN.md 5장이 스키마 계약서다. 이 파일의 목적은 하나다:
//   **공력 담당이 CSV 를 갈아끼우면 코드 수정 없이 반영된다.**
//
// 그래서 파싱이 방어적이다. 실제로 오는 CSV 는 Excel 에서 나오고, Excel 은
// 줄끝에 \r 을 붙이고 파일 앞에 BOM 을 붙인다. 그걸 안 걸러내면 마지막 열이
// 통째로 NaN 이 되고, NaN 은 힘이 되어 기체를 우주로 날려버린다.
#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace fast_drone {

/// 표의 한 칸. (V, alpha) 격자점 하나에 붙는 계수 묶음.
struct Coeffs {
  double C_A = 0.0;    ///< 축력계수 (S_ref 기준, 무차원)
  double C_N = 0.0;    ///< 법선력계수 (S_ref 기준, 무차원)
  double x_cp = 0.0;   ///< 압력중심 [m] — CG 기준 body x. 음수 = CG 뒤 = 안정
  double C_lp = 0.0;   ///< 롤 감쇠 미분계수 (음수여야 함)
  double C_mq = 0.0;   ///< 피치/요 감쇠 미분계수 (음수여야 함). 축대칭이라 공용
};

/// 이 플러그인이 읽을 수 있는 스키마. 다르면 로드를 거부한다.
/// (축대칭 일반화 연구가 만든 표를 v1 이 조용히 읽는 사고 방지 — DESIGN.md 5.2)
inline constexpr const char *kSupportedSchema = "axisym_v1";

class AeroTable {
 public:
  // ── 조회 ────────────────────────────────────────────────────────────
  /// (V, alpha) 에서 계수를 얻는다. 격자 밖은 **가장자리 값으로 고정(clamp)**한다.
  /// 외삽하지 않는 이유: 표 밖에서 외삽한 공력은 대개 발산하고, 발산한 힘은
  /// 조용히 틀리는 게 아니라 요란하게 틀려서 원인을 찾기 어렵다. 고정이 안전하다.
  Coeffs Lookup(double V, double alpha) const {
    size_t iv0, iv1, ia0, ia1;
    double tv, ta;
    Locate(v_grid_, V, &iv0, &iv1, &tv);
    Locate(alpha_grid_, alpha, &ia0, &ia1, &ta);

    const Coeffs &c00 = At(iv0, ia0);
    const Coeffs &c01 = At(iv0, ia1);
    const Coeffs &c10 = At(iv1, ia0);
    const Coeffs &c11 = At(iv1, ia1);

    const double w00 = (1.0 - tv) * (1.0 - ta);
    const double w01 = (1.0 - tv) * ta;
    const double w10 = tv * (1.0 - ta);
    const double w11 = tv * ta;

    Coeffs out;
    out.C_A  = w00*c00.C_A  + w01*c01.C_A  + w10*c10.C_A  + w11*c11.C_A;
    out.C_N  = w00*c00.C_N  + w01*c01.C_N  + w10*c10.C_N  + w11*c11.C_N;
    out.x_cp = w00*c00.x_cp + w01*c01.x_cp + w10*c10.x_cp + w11*c11.x_cp;
    out.C_lp = w00*c00.C_lp + w01*c01.C_lp + w10*c10.C_lp + w11*c11.C_lp;
    out.C_mq = w00*c00.C_mq + w01*c01.C_mq + w10*c10.C_mq + w11*c11.C_mq;
    return out;
  }

  // ── 메타데이터 (CSV 주석줄에서 읽는다) ────────────────────────────────
  double s_ref() const { return s_ref_; }
  double d_ref() const { return d_ref_; }
  double rho_ref() const { return rho_ref_; }
  /// 이 표의 격자가 목표한 힘 오차 [N]. 없으면 0 (검증이 기본값을 쓴다).
  double f_tol_n() const { return f_tol_n_; }
  const std::string &schema() const { return schema_; }
  const std::string &source() const { return source_; }
  size_t n_v() const { return v_grid_.size(); }
  size_t n_alpha() const { return alpha_grid_.size(); }
  const std::vector<double> &v_grid() const { return v_grid_; }
  const std::vector<double> &alpha_grid() const { return alpha_grid_; }

  // ── 로드 ────────────────────────────────────────────────────────────
  static bool Load(const std::string &path, AeroTable *out, std::string *err);

  // ── 겹0: 물리 상식 검사 (DESIGN.md 4장) ──────────────────────────────
  /// CSV 가 이상해도 잡는다. 반환 false = 로드 거부.
  /// warn 에 담기는 것은 "이상하지만 알고 쓰는 것"이라 실행은 시킨다.
  bool SanityCheck(std::vector<std::string> *warn, std::string *err) const;

 private:
  const Coeffs &At(size_t iv, size_t ia) const {
    return data_[iv * alpha_grid_.size() + ia];
  }

  /// 비균일 격자에서 x 를 감싸는 두 인덱스와 보간 가중치를 찾는다.
  ///
  /// 격자점 위에서는 t 가 정확히 0 또는 1 이 되어 **보간 오차가 0**이다.
  /// 이게 검증 test A(1e-9 요구)가 성립하는 근거다.
  static void Locate(const std::vector<double> &g, double x,
                     size_t *i0, size_t *i1, double *t) {
    const size_t n = g.size();
    if (n == 0) { *i0 = *i1 = 0; *t = 0.0; return; }
    if (n == 1) { *i0 = *i1 = 0; *t = 0.0; return; }
    if (!std::isfinite(x)) { *i0 = *i1 = 0; *t = 0.0; return; }

    // upper_bound: g 안에서 x 보다 **큰** 첫 원소의 위치
    auto it = std::upper_bound(g.begin(), g.end(), x);
    size_t hi = static_cast<size_t>(it - g.begin());

    if (hi == 0) {                 // x 가 격자 아래 -> 첫 점으로 고정
      *i0 = 0; *i1 = 1; *t = 0.0;
    } else if (hi >= n) {          // x 가 격자 위 -> 끝 점으로 고정
      *i0 = n - 2; *i1 = n - 1; *t = 1.0;
    } else {
      *i0 = hi - 1; *i1 = hi;
      const double d = g[*i1] - g[*i0];
      *t = (d > 0.0) ? (x - g[*i0]) / d : 0.0;
    }
  }

  std::vector<double> v_grid_;
  std::vector<double> alpha_grid_;
  std::vector<Coeffs> data_;      ///< 행 우선: index = iv * n_alpha + ia
  double s_ref_ = 0.0, d_ref_ = 0.0, rho_ref_ = 0.0, f_tol_n_ = 0.0;
  std::string schema_, source_;
};

// ══════════════════════════════════════════════════════════════════════
// 구현
// ══════════════════════════════════════════════════════════════════════
namespace detail {

/// 줄 끝의 \r(윈도 줄끝)과 앞뒤 공백을 떼어낸다.
inline void TrimInPlace(std::string *s) {
  size_t b = 0, e = s->size();
  while (b < e && ((*s)[b] == ' ' || (*s)[b] == '\t')) ++b;
  while (e > b && ((*s)[e-1] == ' ' || (*s)[e-1] == '\t' ||
                   (*s)[e-1] == '\r' || (*s)[e-1] == '\n')) --e;
  *s = s->substr(b, e - b);
}

/// UTF-8 BOM(Excel 이 붙이는 보이지 않는 3바이트) 제거.
inline void StripBom(std::string *s) {
  if (s->size() >= 3 && static_cast<unsigned char>((*s)[0]) == 0xEF &&
      static_cast<unsigned char>((*s)[1]) == 0xBB &&
      static_cast<unsigned char>((*s)[2]) == 0xBF) {
    *s = s->substr(3);
  }
}

inline std::vector<std::string> SplitCsv(const std::string &line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    TrimInPlace(&cell);
    out.push_back(cell);
  }
  return out;
}

/// std::stod 는 실패 시 예외를 던진다. 플러그인 안에서 예외는 곤란하므로 감싼다.
inline bool ParseDouble(const std::string &s, double *out) {
  if (s.empty()) return false;
  try {
    size_t used = 0;
    const double v = std::stod(s, &used);
    if (used != s.size()) return false;   // "1.2abc" 같은 반쪽 파싱 거부
    if (!std::isfinite(v)) return false;  // NaN/Inf 가 힘이 되면 기체가 날아간다
    *out = v;
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace detail

inline bool AeroTable::Load(const std::string &path, AeroTable *out,
                            std::string *err) {
  auto fail = [err](const std::string &m) {
    if (err) *err = m;
    return false;
  };
  if (!out) return fail("AeroTable::Load: out 이 nullptr");

  std::ifstream f(path);
  if (!f.is_open()) return fail("CSV 를 열 수 없습니다: " + path);

  AeroTable t;
  std::map<std::string, size_t> col;   // 열 이름 -> 인덱스
  bool header_seen = false;
  size_t line_no = 0;

  // 원본 순서를 그대로 담아 두고, 다 읽은 뒤 격자를 조립한다.
  std::vector<double> raw_v, raw_a;
  std::vector<Coeffs> raw_c;

  std::string line;
  while (std::getline(f, line)) {
    ++line_no;
    if (line_no == 1) detail::StripBom(&line);
    detail::TrimInPlace(&line);
    if (line.empty()) continue;

    // ── 주석줄: "# key: value" 형태면 메타데이터로 받는다 ──
    if (line[0] == '#') {
      const size_t colon = line.find(':');
      if (colon == std::string::npos) continue;
      std::string key = line.substr(1, colon - 1);
      std::string val = line.substr(colon + 1);
      detail::TrimInPlace(&key);
      detail::TrimInPlace(&val);
      double d = 0.0;
      if (key == "schema")        t.schema_ = val;
      else if (key == "source")   t.source_ = val;
      else if (key == "S_ref"   && detail::ParseDouble(val, &d)) t.s_ref_ = d;
      else if (key == "d_ref"   && detail::ParseDouble(val, &d)) t.d_ref_ = d;
      else if (key == "rho_ref" && detail::ParseDouble(val, &d)) t.rho_ref_ = d;
      else if (key == "F_tol_N" && detail::ParseDouble(val, &d)) t.f_tol_n_ = d;
      continue;
    }

    std::vector<std::string> cells = detail::SplitCsv(line);

    // ── 첫 비주석줄 = 헤더. 열 이름으로 위치를 잡는다 ──
    // 이름으로 잡는 이유: 공력 담당이 열 순서를 바꿔도 조용히 어긋나지 않는다.
    if (!header_seen) {
      for (size_t i = 0; i < cells.size(); ++i) col[cells[i]] = i;
      static const char *kRequired[] = {"V_mps", "alpha_rad", "C_A", "C_N",
                                        "x_cp", "C_lp", "C_mq"};
      for (const char *r : kRequired) {
        if (col.find(r) == col.end())
          return fail(std::string("CSV 헤더에 필수 열 '") + r + "' 이 없습니다 (" +
                      path + ":" + std::to_string(line_no) + ")");
      }
      header_seen = true;
      continue;
    }

    // ── 데이터줄 ──
    auto get = [&](const char *name, double *dst) -> bool {
      const size_t i = col[name];
      return i < cells.size() && detail::ParseDouble(cells[i], dst);
    };
    double V = 0.0, a = 0.0;
    Coeffs c;
    if (!get("V_mps", &V) || !get("alpha_rad", &a) ||
        !get("C_A", &c.C_A) || !get("C_N", &c.C_N) || !get("x_cp", &c.x_cp) ||
        !get("C_lp", &c.C_lp) || !get("C_mq", &c.C_mq)) {
      return fail("숫자로 못 읽은 칸이 있습니다: " + path + ":" +
                  std::to_string(line_no) + "  [" + line + "]");
    }
    raw_v.push_back(V);
    raw_a.push_back(a);
    raw_c.push_back(c);
  }

  if (!header_seen) return fail("CSV 에 헤더줄이 없습니다: " + path);
  if (raw_c.empty()) return fail("CSV 에 데이터줄이 없습니다: " + path);

  // ── 스키마 확인 (DESIGN.md 5.2) ──
  if (t.schema_ != kSupportedSchema) {
    return fail("스키마가 맞지 않습니다: '" + t.schema_ + "' (이 플러그인은 '" +
                kSupportedSchema + "' 만 읽습니다). 축대칭 일반화 표라면 "
                "일반화 플러그인을 쓰세요");
  }

  // ── 격자 조립 : V 바깥 / alpha 안쪽 순서를 전제한다 ──
  // 첫 V 블록에서 alpha 목록을 뽑고, 이후 모든 블록이 같은지 확인한다.
  for (size_t i = 0; i < raw_v.size(); ++i) {
    if (t.v_grid_.empty() || raw_v[i] != t.v_grid_.back()) {
      // 새 V 블록 시작
      if (!t.v_grid_.empty() && raw_v[i] <= t.v_grid_.back())
        return fail("V 가 증가하지 않습니다 (" + std::to_string(raw_v[i]) +
                    " 다음에 " + std::to_string(t.v_grid_.back()) + "). "
                    "표는 V 바깥/alpha 안쪽 순서로 정렬돼야 합니다");
      t.v_grid_.push_back(raw_v[i]);
    }
  }
  const size_t n_v = t.v_grid_.size();
  if (raw_c.size() % n_v != 0)
    return fail("행 수(" + std::to_string(raw_c.size()) +
                ")가 V 개수(" + std::to_string(n_v) + ")로 나눠지지 않습니다 — "
                "정규격자가 아닙니다");
  const size_t n_a = raw_c.size() / n_v;

  for (size_t j = 0; j < n_a; ++j) t.alpha_grid_.push_back(raw_a[j]);
  for (size_t j = 1; j < n_a; ++j) {
    if (!(t.alpha_grid_[j] > t.alpha_grid_[j-1]))
      return fail("alpha 가 증가하지 않습니다 (index " + std::to_string(j) + ")");
  }
  for (size_t iv = 0; iv < n_v; ++iv) {
    for (size_t ia = 0; ia < n_a; ++ia) {
      const size_t k = iv * n_a + ia;
      if (raw_v[k] != t.v_grid_[iv])
        return fail("정규격자가 아닙니다: 행 " + std::to_string(k) +
                    " 의 V 가 블록과 다릅니다");
      if (raw_a[k] != t.alpha_grid_[ia])
        return fail("정규격자가 아닙니다: 행 " + std::to_string(k) +
                    " 의 alpha 가 첫 블록과 다릅니다 — 모든 V 에서 alpha 목록이 "
                    "같아야 합니다");
    }
  }
  t.data_ = std::move(raw_c);

  *out = std::move(t);
  if (err) err->clear();
  return true;
}

inline bool AeroTable::SanityCheck(std::vector<std::string> *warn,
                                   std::string *err) const {
  auto fail = [err](const std::string &m) {
    if (err) *err = m;
    return false;
  };
  auto note = [warn](const std::string &m) { if (warn) warn->push_back(m); };

  if (s_ref_ <= 0.0) return fail("메타 S_ref 가 없거나 0 이하입니다");
  if (d_ref_ <= 0.0) return fail("메타 d_ref 가 없거나 0 이하입니다");

  const double kHalfPi = 1.5707963267948966;
  bool reverse_flow_positive_ca = false;

  for (size_t iv = 0; iv < v_grid_.size(); ++iv) {
    for (size_t ia = 0; ia < alpha_grid_.size(); ++ia) {
      const Coeffs &c = At(iv, ia);
      const double a = alpha_grid_[ia];

      // 1) alpha = 0 에서 법선력은 0 이어야 한다 (축대칭)
      if (a == 0.0 && std::fabs(c.C_N) > 1e-9)
        return fail("alpha=0 인데 C_N = " + std::to_string(c.C_N) +
                    " 입니다 (0 이어야 함). 축대칭 가정이 깨졌거나 표가 잘못됐습니다");

      // 2) 전방 유동에서 축력은 항력이므로 양수
      if (a < kHalfPi && !(c.C_A > 0.0))
        return fail("alpha=" + std::to_string(a) + " rad 에서 C_A = " +
                    std::to_string(c.C_A) + " 입니다 (전방 유동에서는 양수여야 함)");

      // 3) 감쇠는 항상 omega 를 줄이는 방향
      if (!(c.C_lp < 0.0))
        return fail("C_lp = " + std::to_string(c.C_lp) +
                    " 입니다 (음수여야 함 — 양수면 롤이 발산합니다)");
      if (!(c.C_mq < 0.0))
        return fail("C_mq = " + std::to_string(c.C_mq) +
                    " 입니다 (음수여야 함 — 양수면 피치/요가 발산합니다)");

      // 4) 역류(alpha > 90도)에서 축력이 양수면 물리적으로 뒤로 미는 힘이 된다.
      //    지금 팀 모델이 그렇다 (DESIGN.md 7장 미결). 경고만 하고 돌린다.
      if (a > kHalfPi && c.C_A > 0.0) reverse_flow_positive_ca = true;
    }
  }

  if (reverse_flow_positive_ca) {
    note("alpha > 90도(역류) 격자에서 C_A 가 양수입니다. 그 영역에서 축력이 "
         "기체를 뒤로 밀어 음의 감쇠가 됩니다 (테일시터 호버 강하/천이에서 지나는 "
         "영역). CFD/풍동 값으로 교체 전까지 그 구간 결과는 믿지 마세요. "
         "DESIGN.md 7장 참고");
  }
  if (err) err->clear();
  return true;
}

}  // namespace fast_drone
