# 기체 프로파일 — 왜 이 폴더만 있나

`control` 브랜치에는 `research/` 웹 시뮬레이터 엔진이 없다. 여기 있는 것은
**엔진이 아니라 기체 정의(데이터)** 하나뿐이다.

`selected.json` 은 논문 §5.2 가 지정한 선정 프로파일(질량 1.711714 kg, 선정안 6931)
이고, `control/vehicle_params.py` 의 `load_selected_params()` 가 이 파일을 읽어
`selected_params` 를 **파생**한다(복사가 아니라서 원본이 갱신되면 따라간다).

엔진(JS/CasADi 런타임)은 가져오지 않았다. 두 코드베이스는 언어도 검증
파이프라인도 다르고, 논문 §5.2 가 "공개 데모와 연구용 솔버를 혼합하지 않는다"고
못박았기 때문이다. 공유하는 것은 **기체가 무엇인가** 하나다.

원본: `bulnabi` 브랜치 `research/profiles/selected.json`
