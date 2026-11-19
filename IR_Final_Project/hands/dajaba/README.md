# dajaba_v2 — 새 FINGER 재조립 + 반납 정확도 + 저마찰

`hands/dajaba_v2/` 폴더째로 넣고 실행:

    uv run evaluate_hand.py --hand hands/dajaba_v2/hand.xml --seed 0

시드 0–4 (각 -n 5): 작업 성공 21/25, 평균 보상 0.83 (기존 hand.xml: 1/25, 0.04)

## meshes/
- CHASSIS_BODY.stl / CHASSIS_DISK.stl: CHASSIS_double.stl을 플랜지 원판 윗면(CAD z = -60)에서 나눈 것. 원판은 플랜지에 고정, 나머지(판 + 다리 4개, 다리 5 mm)는 플로팅 스테이지 위 (업로드한 13 mm에서 5 mm로 줄임)
- GEAR, CRANK, COUPLER: 기존 그대로 (CHASSIS.stl은 더 이상 안 씀)
- FINGER_OFFSET.stl: 업로드한 FINGER에서 조(jaw) 막대만 안쪽(파트 -y)으로 9.4 mm 이동. 핀 구멍 A/B, 팔 부분은 그대로
- JAW_V_L.stl: 왼쪽 그립 조 판. 134° V홈 (입구 ±17 mm, 깊이 7.2 mm), 피펫 축 방향은 손가락 두께(15 mm)만큼, 면은 PTFE (μ 0.1)
- JAW_T_R.stl: 오른쪽 그립 조 판. 폭 32 mm, 두께 1.8 mm 평판 텅, 높이 15 mm(손가락 두께), PTFE
- SQZ_PAD_L/R.stl: 스퀴저 PTFE 스트립 (폭 12 mm, 두께 0.8 mm)

모든 핑거/인서트 STL은 업로드한 FINGER.stl과 같은 **파트 좌표계**라서 Inventor 핑거 파트에 바로 겹쳐 볼 수 있음.
(조 판은 핑거 판과 같은 파트 z 0–15 mm 구간에 있음)

## hand.xml에만 있는 수동 메커니즘 (CAD에 추가 필요)
1. 플로팅 스테이지: 그리퍼 두 벌 전체를 손 y·z 방향 교차 리니어 가이드(±16 mm) 위에 올리고, 각 가이드에 스프링 예압 브레이크 패드(frictionloss 1.5 N)
2. 열림 센터링 스토퍼: 그립 크랭크 끝 핀(r 3 mm)이 그리퍼를 열면 플랜지 원판 브래킷의 V노치에 들어가 스테이지를 가운데에 고정 (기어 0.05 rad에서 맞물림)
3. 스테이지 가이드는 원판과 판 사이 다리 구간(5 mm)에 배치
