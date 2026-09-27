# 이 튜닝 실행은 superseded다 (2026-09-27)

**이 폴더의 모든 기록·튜닝값(GSLQR·CPID 포함, 둘 다 status=complete였다)을 쓰지 말 것.**
보고서 `results/ARENA_STATUS_2026-09-26.md` 12절·13절, kj 결정(2026-09-27).

## 무엇이 잘못됐나

이 실행이 쓴 튜닝 시나리오 집합(`configs/arena.json` `tuning.scenarios`, 옛 `tune_*` 8개)은
**제어기 결과를 보고** 저속 사례를 골랐다("사전 게인이 실패 없이 끝낸 후보", 옛 보고서 8.3절).
그 결과 모든 축에서 본시험보다 약한 집합이 됐다(측풍 3·5 vs 본시험 10, 수직풍 방향 반대, 제동·ρ=1
없음). 튜닝값 CPID가 본시험 V_L 측풍 10 m/s에서 실패했다(사전값은 통과) — 튜닝 집합이 본시험
강도를 감싸지 못한 과적합이다(12절, 메모리 `tuning-set-must-match-test-severity` 재발).

## 남은 상태

- V13: complete 120/120 (superseded 튜닝값)
- F13: complete 120/120 (superseded 튜닝값)
- GSLQR: complete 120/120 (superseded 튜닝값)
- CPID: complete 120/120 (superseded 튜닝값)
- M17: **69/120, kj 승인으로 프로세스 중단**(2026-09-27, status는 'running'으로 남아 있다 — 정상
  종료가 아니라 강제 종료다). 이 상태 때문에 `test_i4_committed_pilot_tuning_records_are_fair`는
  이 폴더를 계속 건너뛴다(전부 complete가 아니므로).

## 새 실행

`configs/arena.json`을 kj 결정대로 새 튜닝 시나리오 집합(`tune2_*`, 제어기 결과를 보지 않고
본시험 크기·속도·ρ만으로 정함)으로 바꾼 뒤, 새 run-dir `results/arena/tuning/retune_v2`에서
다시 튜닝한다. 이 폴더(main120)는 "무엇이, 왜 잘못됐는지"의 기록으로만 남긴다 — 지우지 않는다.
