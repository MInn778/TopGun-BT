# TopGun-BT

**팀 TeamKangnam** — [2026 AI Pilot Top Gun Challenge](https://aipilot2026.kau.ac.kr/) 제출작

한국항공대학교 SW중심대학사업단이 주최한 전국 대학(원)생 대상 AI 공중전(Dogfight) 시뮬레이션
대회에 제출한 F-16 1:1 공중전 AI 에이전트입니다. C++로 작성한 행동 트리(Behavior Tree) 기반
전술 엔진을 DLL로 빌드해 대회 측 언리얼 엔진 환경(`DogFightEnv`)에 플러그인 형태로 탑재합니다.

## 대회 개요

- **주최**: 한국항공대학교 SW중심대학사업단
- **형식**: 1:1 Dogfight, 경기 시간 200초
- **승리 조건**: 기총사격으로 상대 격추 시 즉시 승리, 아니면 남은 체력으로 판정
- **교전규칙(ROE)**: 시간대별로 타격각·사정거리가 단계적으로 넓어짐
  - Phase 1 (0~100s): 타격각 2°, 사정거리 500–3,000ft
  - Phase 2 (100~150s): 타격각 4°, 사정거리 500–3,500ft
  - Phase 3 (150~200s): 타격각 6°, 사정거리 500–4,000ft
- **일정**: 예선 2026.08.27–28 (스위스 토너먼트), 본선 2026.09.17
- 참가 접수부터 예선까지 전국 약 80개 대학 290개 팀이 참가, 예선 통과 16개 팀이 본선 진출

## 이 저장소 구성

| 경로 | 설명 |
|---|---|
| `AIP_DCS_TeamKangnam.dll` | 대회 제출용으로 빌드/개명한 BT 두뇌 (원본 파일명 `AIP_DCS_team01.dll`) |
| `Rule_TeamKangnam.xml` | 그 두뇌가 참조하는 전술 트리 정의 (원본 파일명 `Rule_v1.xml`). DLL과 반드시 세트로 사용 |
| `source_AIP_DCS/` | 위 DLL을 만든 C++ 소스 전체 (Visual Studio 프로젝트) |
| `명령어.txt` | 실행/빌드 방법 원본 메모 |
| `추가필요라이브러리_TeamKangnam.txt` | 추가 의존성 여부 (없음 — 표준 MSVC 런타임만 필요) |

> 주의: 프로젝트 폴더에 `AIP_DCS_TeamKangnam.dll`이라는 이름의 파일이 하나 더 있을 수 있는데,
> 그건 팀원이 비교용으로 공유한 별도 BT(소스 없음, 스파링 상대용)이니 혼동하지 말 것. 이 저장소에
> 들어있는 건 `source_AIP_DCS/` 빌드 결과물임.

## 실행 방법

### 1) 대회 제공 환경에서 실행 (권장)

DLL·XML 파일명은 바꾸지 말고, 대회 측 `DogFightEnv/Release/` 폴더에 두 파일을 복사한 뒤 아래
명령으로 실행합니다.

```
python run_unreal_inference.py --mode bt ^
    --bt-dll AIP_DCS_TeamKangnam.dll ^
    --bt-rule-xml Rule_TeamKangnam.xml ^
    --team-name TeamKangnam ^
    --server-ip <서버 IP> ^
    --server-port 9999
```

(DLL이 실행 시 XML을 `Rule_forTraining.xml`이라는 고정 이름으로 복사해서 읽으므로,
`run_unreal_inference.py`가 자동으로 이 복사를 처리합니다. 수동으로 파일명을 바꿀 필요 없음.)

### 2) 소스에서 직접 빌드

`source_AIP_DCS/AIP_DCS.sln`을 Visual Studio(v143 도구 집합, C++ 데스크톱 개발 워크로드)로 열고
Release | x64 구성으로 빌드하면 `AIP_DCS.dll`이 생성됩니다. 이 파일을 `AIP_DCS_TeamKangnam.dll`로
이름만 바꿔서 사용하면 동일한 결과가 나옵니다.

동봉한 `AIP_DCS_TeamKangnam.dll`은 이미 팀 내부 라이브 테스트로 검증된 바이너리이며, 소스에서
새로 빌드한 결과와 바이트 단위로 동일함을 별도 확인하지는 않았으니, 가능하면 동봉된 DLL을 그대로
사용하는 것을 권장합니다.

## 전술 로직 개요 (`Rule_TeamKangnam.xml`)

매 프레임 블랙보드(BB)를 갱신한 뒤, 아래 우선순위의 `Fallback`으로 기동을 결정합니다.

1. **비상 고도 방어막** — 고도 1,500m 미만이면 급강하 회복 기동 우선 실행 (자멸 방지)
2. **WEZ 근접 사격** — 거리 914m 이내면 근접 조준 기동
3. **방어 기동** — 거리 1,100m 이내 + 위협 판정 시 회피
4. **중거리 추격** — 거리 6,000m 이내면 꼬리 잡기(stern-conversion) 기동
5. **원거리 인터셉트** — 거리 6,000m 초과 시 상대 선회각속도를 예측해 코너컷팅으로 접근
6. **기본 유지 기동** — 위 조건에 모두 해당하지 않을 때의 기본값

## 라이선스 / 참고

의존 라이브러리 없음 (순수 C++/MSVC). BehaviorTree 구현은
[BehaviorTree.CPP v3](https://github.com/BehaviorTree/BehaviorTree.CPP) 구조를 참고했습니다.
