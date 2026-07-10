# HANDOFF — 고속 ISR 드론 제어 시뮬레이션

## 프로젝트 목표
축대칭 미사일형 동체 + 쿼드콥터 추진 드론의 고속(300 km/h) 비행 제어.
파이썬 시뮬 완료. 다음 = Ubuntu/Gazebo.

## 확정 사항
- **제어기: ProperHybrid** (VirtualNMPC + INDI 분리) — 통합 미션 전체 1위
- **센서: RTK GPS** (sigma=0.02m) — 표준GPS에서는 고성능 제어 불가
- **안전: HybridWithFallback** — 이상 감지 시 LQR 자동 전환
- **최종 목표: 실기체 비행** (PX4 + Gazebo -> 실기)

## 최종 결과 (통합 미션, 모든 버그 수정 후)

미션: 이륙(10s)->가속(15s)->순항+돌풍(15s)->감속(15s)->호버(7s) = 65초

### 참값 제어

| 제어기 | RMSE vx | RMSE z | 구간 1위 |
|--------|---------|--------|---------|
| LQR | 16.78 | 4.44 | 0/6 |
| **Hybrid** | **1.88** | **1.75** | **5/6** |
| NMPC | 3.73 | 2.80 | 1/6 (감속) |

### RTK EKF

| 제어기 | 참값 z | RTK EKF z | 변화 |
|--------|-------|----------|------|
| LQR | 4.44 | 4.44 | 0% |
| **Hybrid** | **1.75** | **1.67** | **-4.7%** |

### MC 10회: Hybrid 최악(2.82) < LQR 최선(4.41) = 통계적 우위 확실

## 이번 세션 핵심 작업

1. **통합 미션 시뮬 구축** (mission_sim.py)
   - 전 비행구간을 하나로: 개별 시나리오 불필요
   - MC + RTK EKF + 시각화 포함

2. **감속 폭발 발견 및 해결**
   - 원인: INDI G 행렬이 전진비 무시 -> 감속 중 추력 23% 과대평가
   - 수정: dT/dn = k_T * n * (1 + fac) (3-1)
   - 결과: 감속 RMSE z 8.15 -> 1.45 (-82%)
   - RTK EKF 추가: 1.45 -> 0.83 (-43%)

3. **버그 5건 수정**
   - 1-1: 폴백 NaN 검사 위치 (안전)
   - 1-2: RTK R값 ESKF 전달 (RTK 결과 신뢰성)
   - 1-3: NMPC w0 warm start 리셋 (MC 독립성)
   - 2-2: VirtualNMPC T_ref 전달
   - 3-1: INDI G 전진비 반영 (감속 해결)

4. **폴더 정리**
   - legacy/: 이전 스크립트 11개
   - results/: 결과물 (분석.md, 플롯.png, 결과.txt)

## 파일 구조

```
fast_drone/
├── dynamics.py, vehicle_params.py, trim.py     플랜트 핵심
├── controller.py, nmpc.py                      제어기
├── hybrid_comparison.py                        ProperHybrid + VirtualNMPC
├── fallback_controller.py                      Hybrid+LQR 폴백
├── sensors.py, estimator.py, ekf_sim.py        ESKF
├── ekf_comparison.py, gust_comparison.py       유틸
├── mission_sim.py                              통합 미션 (메인 진입점)
├── test_plant.py                               플랜트 테스트
├── results/                                    결과물
│   ├── PROJECT_REPORT.md                       종합 보고서
│   ├── MISSION_ANALYSIS.md                     미션 분석
│   └── FILES.md                                파일 설명
├── legacy/                                     이전 스크립트
├── ros2_ws/                                    ROS2 패키지
└── scripts/                                    Ubuntu 세팅
```

## 다음 단계

### 1순위: Ubuntu/Gazebo 환경 세팅
- Phase 0: Ubuntu 24.04 + ROS2 Jazzy + PX4 + micro-XRCE-DDS + Gazebo Harmonic
- Phase 1: 기본 쿼드(x500)로 SITL 통신 확인
- scripts/setup_ubuntu.sh 준비 완료

### 2순위: Gazebo 커스텀 통합
- 커스텀 공력 플러그인 (C++)
- controllers/ 디렉토리에 제어기 복사

### 3순위: 실기체
- RTK GPS 모듈 선정
- 컴패니언 컴퓨터 + acados RTI

## 주의사항

- NMPC/VirtualNMPC: reset() 시 _last_t + w0 초기화 필수
- INDI G: 전진비(advance ratio) 반영 필수 (compute_control_effectiveness)
- ESKF: update_gps에 실제 R_pos/R_vel 전달 필수
- 폴백: NaN 검사는 채터링 가드보다 항상 우선
- 감속 잔존 이슈: MC 3%에서 20m+ (폴백으로 대응)
- NMPC 단독 실기 부적합 (느리고 센서 취약)
