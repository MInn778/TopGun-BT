#include "Task_MidRangeEngage.h"
#include <cmath>
#include <iostream>

namespace
{
	const double PI = 3.14159265358979323846;
	const double OMEGA_EPS = 1e-4;
	const double MAX_DT_FOR_RATE = 0.5;
	const double SMOOTH_ALPHA = 0.25;
	// 2026-08-21(라이브 로그 기반): Task_Intercept.cpp와 동일한 버그 -- Omega
	// 추정치가 노이즈(직선비행 중에도 ±0.5~3도/초)인데 OMEGA_EPS가 너무 작아
	// 거의 항상 "선회 중"으로 오판정, V/Omega가 분모 노이즈로 폭주해서 조준점이
	// 실제 거리보다 몇 배(실측: 최대 6538m 실거리에 distVP 33000m대) 멀리
	// 튐. Task_Intercept와 동일하게 회전반경 자체로 게이트한다.
	// 2026-08-22(근본원인 조사, 시도했다 되돌림 -- 근거는 Task_Intercept.cpp
	// 주석 참고): 5000m로 낮췄더니 거리는 훨씬 잘 좁혀졌지만 60판 배치
	// 순마진이 0->-14로 이번 세션 최악의 후퇴(안 붙는 게 의도치 않은
	// 생존전략이었음, Task_WEZAttack의 근접전 조준 문제가 먼저 풀려야 함).
	const double MAX_SANE_TURN_RADIUS = 20000.0;

	double WrapToPi(double angle)
	{
		while (angle > PI) angle -= 2.0 * PI;
		while (angle < -PI) angle += 2.0 * PI;
		return angle;
	}
}

namespace Action
{
	PortsList Task_MidRangeEngage::providedPorts()
	{
		return {
				InputPort<CPPBlackBoard*>("BB")
		};
	}

	NodeStatus Task_MidRangeEngage::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		// 2026-08-16 (3차 튜닝): closure-time 리드로 목표 앞을 가로지르는 궤적은
		// 실측 결과 WEZ 사거리 체류가 0.6%밖에 안 나왔다(스쳐 지나가기만 함) --
		// 대신 상대의 "6시 방향"(꼬리)을 조준점으로 삼아 선회해 들어가는
		// stern conversion(후미 전환) 기동으로 교체. 목표와 같은 진행 방향/속도로
		// 붙는 궤적이라 WEZAttack 인수인계 시점에 상대 속도가 낮고(체류시간 유리),
		// 상대에게 진 AA도 자연히 좋아진다(꼬리를 잡으러 가는 형태이므로).
		// trail 거리는 거리 비례로 줄여서 914m WEZ 경계에서 자연스럽게 순수 추적에 가까워짐.
		float trailDistance = (float)((*BB)->Distance * 0.4);
		trailDistance = trailDistance < 150.0f ? 150.0f : (trailDistance > 900.0f ? 900.0f : trailDistance);

		// 2026-08-20(신규): 위 "6시 방향" 조준점이 상대의 "지금" 위치+진행방향
		// 으로만 계산돼서, 상대가 선회 중이면 조준점도 계속 따라 회전하며
		// 도망감(실측: 내부 LOS가 100~150도에서 안 줄어듦, 상대를 계속 등 뒤에
		// 두고 넓게 도는 패턴) -- Task_Intercept.h의 문제의식과 동일("chasing
		// a point on the EDGE of the target's turn circle keeps pushing the
		// pursuer wide"). 주의: 3차 튜닝에서 실패한 "목표 앞을 가로지르는
		// 리드"와는 다르다 -- 그건 상대 정면 쪽으로 미리 아이밍해서 충돌
		// 코스가 됐던 것이고, 여기는 여전히 "미래 시점의 6시 방향"(뒤쪽)을
		// 조준하므로 같은 함정이 아님. Task_Intercept와 동일한 등속 원운동
		// 추정으로 상대의 짧은 미래(리드타임 상한 6초, 원거리보다 훨씬 짧게
		// 캡 -- 이 거리대는 원래 빨리 수렴해야 함) 위치/방향을 예측해서, 그
		// 시점의 꼬리를 조준한다.
		Vector3 TargetForward = (*BB)->TargetForwardVector;
		TargetForward.normalize();

		double RunningTime = (*BB)->RunningTime;
		double Heading = std::atan2(TargetForward.Y, TargetForward.X);

		if (m_HasPrevHeading)
		{
			double dt = RunningTime - m_PrevRunningTime;
			if (dt > 0.0 && dt < MAX_DT_FOR_RATE)
			{
				double dHeading = WrapToPi(Heading - m_PrevHeading);
				double rawOmega = dHeading / dt;
				m_OmegaFiltered = m_OmegaFiltered + SMOOTH_ALPHA * (rawOmega - m_OmegaFiltered);
			}
			else
			{
				m_OmegaFiltered = 0.0;
			}
		}
		m_PrevHeading = Heading;
		m_PrevRunningTime = RunningTime;
		m_HasPrevHeading = true;

		double V = (double)(*BB)->TargetSpeed_MS;
		double Omega = m_OmegaFiltered;

		double mySpeed = (double)(*BB)->MySpeed_MS;
		if (mySpeed < 50.0) mySpeed = 50.0; // 저속/0 나눗셈 방지
		double leadTime = (double)(*BB)->Distance / mySpeed;
		if (leadTime > 6.0) leadTime = 6.0;   // 원거리(Intercept)보다 훨씬 짧게 캡
		if (leadTime < 0.3) leadTime = 0.3;   // 근접 시 최소한의 반응 여유는 남김

		bool isTurning = std::fabs(Omega) >= OMEGA_EPS && (V / std::fabs(Omega)) <= MAX_SANE_TURN_RADIUS;

		double futureHeading = Heading + Omega * leadTime;
		double leadDx, leadDy;
		if (!isTurning)
		{
			leadDx = V * std::cos(Heading) * leadTime;
			leadDy = V * std::sin(Heading) * leadTime;
		}
		else
		{
			leadDx = (V / Omega) * (std::sin(futureHeading) - std::sin(Heading));
			leadDy = -(V / Omega) * (std::cos(futureHeading) - std::cos(Heading));
		}
		double leadDz = TargetForward.Z * V * leadTime;
		Vector3 FuturePos = (*BB)->TargetLocaion_Cartesian + Vector3(leadDx, leadDy, leadDz);

		// 미래 시점의 진행방향(꼬리 오프셋 기준축도 같이 회전시켜야 함 --
		// 안 그러면 위치만 미래고 "꼬리 방향"은 여전히 지금 방향이 되어 버림).
		Vector3 FutureForward((float)std::cos(futureHeading), (float)std::sin(futureHeading), TargetForward.Z);
		FutureForward.normalize();

		Vector3 SternPoint = FuturePos - FutureForward * trailDistance;

		// 2026-08-21(전술 재설계): "꼬리(6시 방향)를 쫓는" 방식은 선회율이 상대와
		// 비슷하면(실측: 우리 1.33도/초 vs 상대 1.50도/초, 거의 동률) 추적곡선이
		// 원리적으로 안 좁혀짐(각속도/속도 우위가 있어야 수렴). own_offaxis가
		// 라이브에서 계속 안 좁혀지던 근본 원인으로 지목됨. Task_Intercept와
		// 동일한 코너컷팅을 도입 -- 상대 선회 반경 밖에서는 선회 중심(Center)을
		// 조준해서 각속도 우위 없이도 기하학적으로 수렴 가능하게 하고, 반경
		// 안쪽(WEZ 진입 직전)으로 좁혀지면 기존 꼬리조준으로 자연히 전환된다.
		if (isTurning)
		{
			double VoverOmega = V / Omega;
			double Cx = (*BB)->TargetLocaion_Cartesian.X - VoverOmega * std::sin(Heading);
			double Cy = (*BB)->TargetLocaion_Cartesian.Y + VoverOmega * std::cos(Heading);
			double Cz = (*BB)->TargetLocaion_Cartesian.Z + leadDz;
			Vector3 Center(Cx, Cy, Cz);

			double R = std::fabs(VoverOmega);
			if (R < 1.0) R = 1.0;

			double d = ((*BB)->MyLocation_Cartesian - Center).length();
			// 2026-08-21(실측 기반 수정): CutoffMult=3.0이면 d/R이 3배는 되어야
			// 코너컷팅이 100%인데, 실측(단판 디버그)으로 보니 실제 교전 중
			// d/R이 거의 항상 1.0~1.5배 수준이라 alpha가 대부분 0.03~0.2에
			// 그침(=97~80%가 여전히 옛날 꼬리조준) -- 코너컷팅을 켰는데도
			// 사실상 안 켠 것과 같았음. 실제 교전거리대에서 의미있게
			// 작동하도록 낮춘다.
			const double CutoffMult = 1.15;
			double alpha = (d / R - 1.0) / (CutoffMult - 1.0);
			if (alpha < 0.0) alpha = 0.0;
			if (alpha > 1.0) alpha = 1.0;

			(*BB)->VP_Cartesian = Center * alpha + SternPoint * (1.0 - alpha);
		}
		else
		{
			(*BB)->VP_Cartesian = SternPoint;
		}

		// 2026-08-21(Task_Intercept.cpp와 동일 원인/수정): leadDz 고도 외삽에
		// 지면 하한이 없어 SternPoint/Center의 Z가 지하까지 튈 수 있음(실측
		// 근거는 Task_Intercept.cpp 주석 참고, 동일한 leadDz 구조).
		if ((*BB)->VP_Cartesian.Z < 50.0) (*BB)->VP_Cartesian.Z = 50.0;

		// 2026-08-20(19차): Task_LongRangeApproach.cpp와 동일 -- 저고도(2000m 밑)
		// 스로틀을 최대 50%까지 줄여 에너지를 낮춰서 조종면 반응성을 회복시킨다.
		// 실측: 롤 교정 명령은 방향이 맞는데도 계속 100% 스로틀(380~420m/s)로
		// 2000m 가까이 거의 안 먹히다가 600m 밑에서야 반응하는 패턴 확인.
		double throttleUrgency = 0.0;
		if ((*BB)->MyLocation_Cartesian.Z < 2000.0)
		{
			throttleUrgency = (2000.0 - (*BB)->MyLocation_Cartesian.Z) / 2000.0;
			if (throttleUrgency > 1.0) throttleUrgency = 1.0;
		}
		(*BB)->Throttle = (float)(1.0 - 0.5 * throttleUrgency);

		return NodeStatus::SUCCESS;
	}
}
