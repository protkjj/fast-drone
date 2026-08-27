// gz-sim system 플러그인 — 검증된 공력 코어를 Gazebo 에 붙이는 얇은 래퍼.
//
// 이 파일은 **계산을 하지 않는다.** 계산은 include/fast_drone/aero_core.hpp 에
// 있고 이미 파이썬과 기계정밀도로 대조됐다 (DESIGN.md 4.4). 여기가 하는 일은
// 네 가지뿐이다:
//   1. SDF 를 읽고 표를 로드한다 (틀렸으면 힘을 하나도 걸지 않고 죽는다)
//   2. 링크의 속도·각속도·자세를 월드에서 읽어 동체(B) 좌표로 옮긴다
//   3. 코어를 부른다
//   4. 결과를 다시 월드로 옮겨 링크에 건다
//
// 좌표 변환(2, 4)이 이 파일의 전부이자 최대 위험이다. DESIGN.md 2장이 계약서다.
//
// ── C++ 처음 보는 사람을 위한 메모 ─────────────────────────────────────────
//  * gz-sim 플러그인은 "System" 이라는 인터페이스를 구현한 클래스다.
//    ISystemConfigure : 월드 로드 때 딱 한 번 불린다 (초기화)
//    ISystemPreUpdate : 물리 스텝마다 불린다. 힘은 여기서 건다
//    `override` 키워드는 "이 함수는 부모의 가상함수를 덮어쓴다" 는 선언이다.
//    오타로 다른 함수를 만들면 컴파일 에러가 나서 조용한 실패를 막아 준다.
//  * `GZ_ADD_PLUGIN` 매크로가 이 클래스를 .so 밖으로 노출시킨다. 이게 없으면
//    빌드는 되는데 Gazebo 가 못 찾는다.
//  * `_ecm` (EntityComponentManager) 는 시뮬 안의 모든 상태가 들어 있는 창고다.
//    링크의 자세·속도를 여기서 꺼내고, 힘도 여기에 써 넣는다.
// ──────────────────────────────────────────────────────────────────────────
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <sdf/Element.hh>

#include "fast_drone/aero_core.hpp"
#include "fast_drone/aero_table.hpp"
#include "fast_drone/frame.hpp"

namespace fast_drone {

namespace gzs = gz::sim;
using gz::math::Vector3d;

class AeroPlugin : public gzs::System,
                   public gzs::ISystemConfigure,
                   public gzs::ISystemPreUpdate {
 public:
  void Configure(const gzs::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gzs::EntityComponentManager &_ecm,
                 gzs::EventManager &_eventMgr) override;

  void PreUpdate(const gzs::UpdateInfo &_info,
                 gzs::EntityComponentManager &_ecm) override;

 private:
  // SDF 도우미 — 없으면 기본값. 있는데 못 읽으면 false 를 돌려 Configure 를 세운다.
  static bool ReadVec3(const std::shared_ptr<const sdf::Element> &_sdf,
                       const char *_name, Vector3d *_out) {
    if (!_sdf->HasElement(_name)) return true;         // 없는 건 정상 (기본값 유지)
    *_out = _sdf->Get<Vector3d>(_name, *_out).first;
    return true;
  }

  void OpenDebugCsv();
  void LogDebugCsv(double _t, const gz::math::Pose3d &_pose,
                   const Vector3d &_vAirW, const Vector3d &_omegaW,
                   const Vec3 &_vB, const Vec3 &_wB, const AeroDebug &_dbg,
                   const Wrench &_wr, const Vector3d &_fW, const Vector3d &_mW);

  gzs::Model model_{gzs::kNullEntity};
  gzs::Entity link_entity_{gzs::kNullEntity};
  std::string link_name_;

  AeroTable table_;
  BodyParams bp_;
  FrameLB frame_;

  Vector3d wind_world_{0.0, 0.0, 0.0};   ///< 월드 ENU 바람 [m/s]

  /// Configure 가 성공했을 때만 true. 실패하면 **힘을 하나도 걸지 않는다.**
  /// 반쯤 설정된 채로 절반만 맞는 힘을 거는 것이 가장 나쁜 실패다.
  bool valid_ = false;

  // ── 검증 훅 (DESIGN.md 4장 겹2) ──
  //
  // 링크의 실제 속도 대신 지정한 값을 쓴다. 상태를 정확히 통제하려고 둔다.
  // offboard_node.py 의 test_motor 훅과 같은 성격이다.
  //
  // ★ **월드(ENU) 좌표**로 받는다. 동체 좌표로 받으면 정작 겹2 가 검증하려던
  //   월드->링크->동체 변환을 건너뛰게 되어 시험이 무의미해진다.
  //   기체를 <static>true</static> 로 두면 자세가 고정돼 조건이 완전히 통제된다.
  bool override_state_ = false;
  Vector3d test_vel_world_{0, 0, 0};    ///< 월드 ENU 무게중심 속도 [m/s]
  Vector3d test_omega_world_{0, 0, 0};  ///< 월드 ENU 각속도 [rad/s]

  std::string table_source_;
  std::string debug_csv_path_;
  std::ofstream debug_csv_;
  int debug_every_ = 1;
  long step_ = 0;
};

// ══════════════════════════════════════════════════════════════════════
void AeroPlugin::Configure(const gzs::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gzs::EntityComponentManager &_ecm,
                           gzs::EventManager &) {
  model_ = gzs::Model(_entity);
  if (!model_.Valid(_ecm)) {
    gzerr << "[fast_drone_aero] 모델에 붙지 않았습니다. <model> 안에 두세요.\n";
    return;
  }
  const std::string mname = model_.Name(_ecm);

  // ── 1) 링크 ──────────────────────────────────────────────────────────
  link_name_ = _sdf->Get<std::string>("link_name", std::string("base_link")).first;
  link_entity_ = model_.LinkByName(_ecm, link_name_);
  if (link_entity_ == gzs::kNullEntity) {
    gzerr << "[fast_drone_aero] '" << mname << "' 에 링크 '" << link_name_
          << "' 가 없습니다. <link_name> 을 확인하세요.\n";
    return;
  }

  // ── 2) 표 ────────────────────────────────────────────────────────────
  const std::string csv_rel = _sdf->Get<std::string>("csv_file", std::string()).first;
  if (csv_rel.empty()) {
    gzerr << "[fast_drone_aero] <csv_file> 이 없습니다. 공력 계수 표가 있어야 합니다.\n";
    return;
  }
  // 상대경로는 이 SDF 파일 기준으로 푼다 (모델 폴더 안에 CSV 를 두는 게 자연스럽다)
  std::string csv_path = gzs::asFullPath(csv_rel, _sdf->FilePath());
  std::string err;
  if (!AeroTable::Load(csv_path, &table_, &err)) {
    // asFullPath 가 엉뚱하게 풀렸을 수 있으니 원문 그대로도 한 번 시도한다
    if (!AeroTable::Load(csv_rel, &table_, &err)) {
      gzerr << "[fast_drone_aero] 표 로드 실패: " << err << "\n"
            << "  시도한 경로: " << csv_path << " / " << csv_rel << "\n";
      return;
    }
    csv_path = csv_rel;
  }

  // ── 3) 기준량 ────────────────────────────────────────────────────────
  // 기본값을 표에서 가져온다. SDF 에 따로 적으면 그게 이긴다.
  bp_.s_ref = _sdf->Get<double>("s_ref", table_.s_ref()).first;
  bp_.d_ref = _sdf->Get<double>("d_ref", table_.d_ref()).first;
  bp_.rho   = _sdf->Get<double>("rho", table_.rho_ref() > 0.0
                                           ? table_.rho_ref() : 1.225).first;
  if (!CheckRefConsistency(table_, bp_, &err)) {
    gzerr << "[fast_drone_aero] " << err << "\n";
    return;
  }

  // ── 4) 겹0 상식 검사 ─────────────────────────────────────────────────
  std::vector<std::string> warn;
  if (!table_.SanityCheck(&warn, &err)) {
    gzerr << "[fast_drone_aero] 표 상식 검사 실패: " << err << "\n";
    return;
  }
  for (const auto &w : warn) gzwarn << "[fast_drone_aero] " << w << "\n";

  // ── 5) 좌표계 (DESIGN.md 2.2) ────────────────────────────────────────
  Vector3d nose{1, 0, 0}, right{0, -1, 0};
  ReadVec3(_sdf, "nose", &nose);
  ReadVec3(_sdf, "right", &right);
  if (!BuildFrameLB({{nose.X(), nose.Y(), nose.Z()}},
                    {{right.X(), right.Y(), right.Z()}}, &frame_, &err)) {
    gzerr << "[fast_drone_aero] 좌표계 설정 오류: " << err << "\n";
    return;
  }

  // ── 6) 바람 · 검증 훅 ────────────────────────────────────────────────
  ReadVec3(_sdf, "wind", &wind_world_);
  if (_sdf->HasElement("test_velocity_world") ||
      _sdf->HasElement("test_omega_world")) {
    override_state_ = true;
    ReadVec3(_sdf, "test_velocity_world", &test_vel_world_);
    ReadVec3(_sdf, "test_omega_world", &test_omega_world_);
    gzwarn << "[fast_drone_aero] ★ 검증 모드 — 링크의 실제 속도를 무시하고 "
              "SDF 에 적힌 월드 속도를 씁니다. 비행 시험에 쓰면 안 됩니다. "
              "v=" << test_vel_world_ << " w=" << test_omega_world_ << "\n";
  }
  table_source_ = csv_path;
  debug_csv_path_ = _sdf->Get<std::string>("debug_csv", std::string()).first;
  debug_every_ = _sdf->Get<int>("debug_csv_every", 1).first;
  if (debug_every_ < 1) debug_every_ = 1;
  if (!debug_csv_path_.empty()) OpenDebugCsv();

  // ── 7) 속도 조회 켜기 ────────────────────────────────────────────────
  // 이걸 안 하면 WorldLinearVelocity 가 아무 값도 안 준다 (gz-sim 기본이 꺼짐).
  gzs::Link link(link_entity_);
  link.EnableVelocityChecks(_ecm, true);

  valid_ = true;
  gzmsg << "[fast_drone_aero] '" << mname << "/" << link_name_ << "' 에 부착\n"
        << "  표    " << csv_path << "  (" << table_.schema() << ", V "
        << table_.n_v() << " x alpha " << table_.n_alpha() << ")\n"
        << "  기준  S_ref " << bp_.s_ref << " m^2, d_ref " << bp_.d_ref
        << " m, rho " << bp_.rho << " kg/m^3\n"
        << "  좌표  nose(" << nose << ") right(" << right << ") -> down("
        << frame_.down[0] << " " << frame_.down[1] << " " << frame_.down[2]
        << ")\n"
        << "  바람  " << wind_world_ << " m/s (월드 ENU)\n";
}

// ══════════════════════════════════════════════════════════════════════
void AeroPlugin::PreUpdate(const gzs::UpdateInfo &_info,
                           gzs::EntityComponentManager &_ecm) {
  if (!valid_ || _info.paused) return;

  gzs::Link link(link_entity_);

  const auto pose = link.WorldPose(_ecm);
  if (!pose) return;                       // 아직 컴포넌트가 안 만들어진 첫 스텝

  Vector3d v_cm_world{0, 0, 0}, omega_world{0, 0, 0};

  if (override_state_) {
    // 검증 모드: 월드 속도를 그대로 주입한다. 아래 변환은 **전부 실제 경로**다.
    v_cm_world = test_vel_world_;
    omega_world = test_omega_world_;
  } else {
    const auto v_origin_w = link.WorldLinearVelocity(_ecm);
    const auto omega_w = link.WorldAngularVelocity(_ecm);
    if (!v_origin_w || !omega_w) return;
    omega_world = *omega_w;

    // 공력은 **무게중심**의 속도로 계산한다. WorldLinearVelocity 는 링크
    // **원점**의 속도라, 회전 중이면 둘이 다르다:  v_cm = v_o + w x (R * r_cm)
    // gz-sim 에 오프셋 속도 오버로드가 있지만 버전마다 있고 없고 해서 직접 쓴다.
    // 두 줄이고 무엇을 하는지가 눈에 보인다.
    Vector3d r_cm_link{0, 0, 0};
    if (auto *inertial = _ecm.Component<gzs::components::Inertial>(link_entity_))
      r_cm_link = inertial->Data().Pose().Pos();
    v_cm_world = *v_origin_w + omega_world.Cross(pose->Rot().RotateVector(r_cm_link));
  }

  // 대기속도 = 기체 속도 - 바람 (둘 다 월드 ENU)
  const Vector3d v_air_world = v_cm_world - wind_world_;

  // 월드 -> 링크(L). RotateVectorReverse 가 R^T v 다.
  const Vector3d v_L = pose->Rot().RotateVectorReverse(v_air_world);
  const Vector3d w_L = pose->Rot().RotateVectorReverse(omega_world);

  // 링크(L) -> 동체(B). 여기가 DESIGN.md 2.2 의 계약.
  const Vec3 v_B = frame_.ToBody({{v_L.X(), v_L.Y(), v_L.Z()}});
  const Vec3 w_B = frame_.ToBody({{w_L.X(), w_L.Y(), w_L.Z()}});

  // ── 검증된 코어 호출 ─────────────────────────────────────────────────
  AeroDebug dbg;
  const Wrench wr = ComputeBodyWrench(table_, bp_, v_B, w_B, &dbg);

  // ── 동체(B) -> 링크(L) -> 월드 ───────────────────────────────────────
  const Vec3 f_L = frame_.ToLink(wr.force);
  const Vec3 m_L = frame_.ToLink(wr.moment);
  const Vector3d f_world =
      pose->Rot().RotateVector(Vector3d(f_L[0], f_L[1], f_L[2]));
  const Vector3d m_world =
      pose->Rot().RotateVector(Vector3d(m_L[0], m_L[1], m_L[2]));

  // ── 적용 ─────────────────────────────────────────────────────────────
  // ★ 적용점이 이 파일에서 두 번째로 위험한 자리다.
  //   gz-sim 의 ExternalWorldWrenchCmd 는 힘을 **링크 원점**에 건다.
  //   그런데 우리 모멘트는 **무게중심 기준**이다 (x_cp 가 CG 기준이므로).
  //   그래서:
  //     - 힘은 AddWorldForce 로 건다. 이 함수가 원점/CG 차이를 알아서 보정한다
  //       (내부에서 r_cm x F 를 토크에 더해 준다).
  //     - 감쇠·정적 모멘트는 **힘이 0 인 순수 우력**으로 건다. 우력은 자유벡터라
  //       적용점이 아예 의미가 없다 -> 원점/CG 모호성이 사라진다.
  //   이 가정은 겹2(Gazebo 실측)에서 확인한다.
  link.AddWorldForce(_ecm, f_world);
  link.AddWorldWrench(_ecm, Vector3d::Zero, m_world);

  if (debug_csv_.is_open() && (step_ % debug_every_ == 0)) {
    const double t = std::chrono::duration<double>(_info.simTime).count();
    LogDebugCsv(t, *pose, v_air_world, omega_world, v_B, w_B, dbg, wr,
                f_world, m_world);
  }
  ++step_;
}

// ══════════════════════════════════════════════════════════════════════
void AeroPlugin::OpenDebugCsv() {
  debug_csv_.open(debug_csv_path_);
  debug_csv_.precision(17);
  if (!debug_csv_.is_open()) {
    gzwarn << "[fast_drone_aero] 디버그 CSV 를 못 엽니다: " << debug_csv_path_
           << "\n";
    return;
  }
  // 설정을 로그 머리에 박아 둔다 — 그래야 겹2 대조 스크립트가 이 파일 하나만
  // 보고 기준값을 만들 수 있다. 설정을 따로 넘기면 언젠가 어긋난다.
  debug_csv_ << "# fast_drone aero debug log\n"
             << "# table: " << table_source_ << "\n"
             << "# S_ref: " << bp_.s_ref << "\n"
             << "# d_ref: " << bp_.d_ref << "\n"
             << "# rho: " << bp_.rho << "\n"
             << "# nose: " << frame_.nose[0] << " " << frame_.nose[1] << " "
             << frame_.nose[2] << "\n"
             << "# right: " << frame_.right[0] << " " << frame_.right[1] << " "
             << frame_.right[2] << "\n"
             << "# down: " << frame_.down[0] << " " << frame_.down[1] << " "
             << frame_.down[2] << "\n"
             << "# wind: " << wind_world_.X() << " " << wind_world_.Y() << " "
             << wind_world_.Z() << "\n"
             << "# quaternion: scalar-last (qx,qy,qz,qw), 링크 -> 월드\n"
             << "# u,v,w,p,q,r : 동체(B, FRD) 좌표\n"
             << "# Fx..Mz      : 동체(B) 좌표. 겹1 기준값과 같은 프레임이다\n"
             << "# fWx..mWz    : 월드(ENU) 좌표. 실제로 링크에 건 값\n"
             << "# qx..qw      : 링크 자세 (월드->링크). vaWx.. : 월드 대기속도\n"
             << "t,qx,qy,qz,qw,vaWx,vaWy,vaWz,owWx,owWy,owWz,"
             << "u,v,w,p,q,r,V,V_cf,alpha,q_bar,C_A,C_N,x_cp,"
             << "Fx,Fy,Fz,Mx,My,Mz,fWx,fWy,fWz,mWx,mWy,mWz\n";
  gzmsg << "[fast_drone_aero] 디버그 CSV: " << debug_csv_path_ << " (매 "
        << debug_every_ << " 스텝)\n";
}

void AeroPlugin::LogDebugCsv(double _t, const gz::math::Pose3d &_pose,
                             const Vector3d &_vAirW, const Vector3d &_omegaW,
                             const Vec3 &_vB, const Vec3 &_wB,
                             const AeroDebug &_dbg, const Wrench &_wr,
                             const Vector3d &_fW, const Vector3d &_mW) {
  const auto &q = _pose.Rot();
  debug_csv_ << _t
             << ',' << q.X() << ',' << q.Y() << ',' << q.Z() << ',' << q.W()
             << ',' << _vAirW.X() << ',' << _vAirW.Y() << ',' << _vAirW.Z()
             << ',' << _omegaW.X() << ',' << _omegaW.Y() << ',' << _omegaW.Z()
             << ',' << _vB[0] << ',' << _vB[1] << ',' << _vB[2]
             << ',' << _wB[0] << ',' << _wB[1] << ',' << _wB[2]
             << ',' << _dbg.V << ',' << _dbg.V_cf << ',' << _dbg.alpha
             << ',' << _dbg.q_bar << ',' << _dbg.c.C_A << ',' << _dbg.c.C_N
             << ',' << _dbg.c.x_cp
             << ',' << _wr.force[0] << ',' << _wr.force[1] << ',' << _wr.force[2]
             << ',' << _wr.moment[0] << ',' << _wr.moment[1] << ',' << _wr.moment[2]
             << ',' << _fW.X() << ',' << _fW.Y() << ',' << _fW.Z()
             << ',' << _mW.X() << ',' << _mW.Y() << ',' << _mW.Z() << '\n';
}

}  // namespace fast_drone

GZ_ADD_PLUGIN(fast_drone::AeroPlugin, gz::sim::System,
              fast_drone::AeroPlugin::ISystemConfigure,
              fast_drone::AeroPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(fast_drone::AeroPlugin, "fast_drone::AeroPlugin")
