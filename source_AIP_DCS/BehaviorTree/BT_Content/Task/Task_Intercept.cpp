#include "Task_Intercept.h"
#include <cmath>

namespace
{
	const double PI = 3.14159265358979323846;
	const double OMEGA_EPS = 1e-4;
	const double MAX_DT_FOR_RATE = 0.5;
	const double SMOOTH_ALPHA = 0.25;
	const double MIN_RADIUS = 1.0;
	// 2026-08-21(라이브 로그 정량분석 기반 수정): OMEGA_EPS(1e-4 rad/s, 약
	// 0.006도/초)가 실측 노이즈 수준(헤딩변화율 추정치가 직선비행 중에도
	// ±0.5~3도/초로 흔들림)보다 훨씬 작아서, 사실상 거의 항상 "선회 중"으로
	// 오판정됨. 그러면 R=V/Omega(분모가 노이즈)가 틱마다 5.7km~539만m까지
	// 요동치고, 그 R로 잡는 코너컷팅 중심점(Center)이 매틱 완전히 다른(때론
	// 정반대) 방향으로 튐 -- 라이브 실측: 적은 계속 우리를 정면(±15도 이내)에
	// 두는데 우리는 계속 적을 후방(±180도 근처)에 두는 현상의 직접 원인이었음.
	// Omega(분모) 대신 실제 물리량인 R 자체로 게이트한다 -- 이 반경보다 크게
	// 도는(=사실상 직진과 다름없는) 경우는 안정적인 직선 리드 공식을 쓴다.
	// 2026-08-22(근본원인 조사, 시도했다 되돌림): 5000m로 낮춰서 alpha=60/90
	// 구간에서 실제로 거리가 훨씬 잘 좁혀지는 것까지는 확인함(단판: 최근접
	// 3028m->991m, 2246m->927m, 심지어 90/seed0은 WEZ 안에 73초 체류).
	// 하지만 60판 배치 결과 순마진 0->-14로 이번 세션 최악의 후퇴(alpha=90
	// 5패, 자멸 2건 신규) -- "안 붙는 것"이 의도치 않게 생존전략이었던 것으로
	// 판명(Task_WEZAttack.cpp의 기존 메모대로 WEZ 안에서 유효 조준이 잘 안
	// 걸려서, 억지로 더 붙게 만들수록 손해만 늘어남). 되돌림 -- 다음은 이
	// 반경게이트보다 Task_WEZAttack의 근접전 조준(ATA) 문제를 먼저 풀 것.
	const double MAX_SANE_TURN_RADIUS = 20000.0;

	double WrapToPi(double angle)
	{
		while (angle > PI) angle -= 2.0 * PI;
		while (angle < -PI) angle += 2.0 * PI;
		return angle;
	}
}

PortsList Action::Task_Intercept::providedPorts()
{
	return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<std::string>("LeadTime", "1.5", "seconds ahead to predict the merge point once close to the turn circle"),
			InputPort<std::string>("CutoffRadiusMult", "3.0",
				"beyond this multiple of the target's turn radius, aim fully at the turn CENTER "
				"(cut the corner) instead of the target itself; blends linearly down to 1x radius")
	};
}

NodeStatus Action::Task_Intercept::tick()
{
	Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
	Optional<std::string> LeadTimeStr = getInput<std::string>("LeadTime");
	Optional<std::string> CutoffMultStr = getInput<std::string>("CutoffRadiusMult");

	double LeadTime = LeadTimeStr ? std::stod(LeadTimeStr.value()) : 1.5;
	double CutoffMult = CutoffMultStr ? std::stod(CutoffMultStr.value()) : 3.0;
	if (CutoffMult <= 1.0) CutoffMult = 1.0001;

	Vector3 TargetForward = (*BB)->TargetForwardVector;
	TargetForward.normalize();

	double RunningTime = (*BB)->RunningTime;
	double Heading = std::atan2(TargetForward.Y, TargetForward.X);

	// Estimate the target's turn rate from measured heading change (same approach as
	// Task_Pure/Task_Lag) rather than guessing the sign from bank angle.
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

	// 2026-08-20: LeadTime 포트값(XML="1.5")은 원거리(6000m 초과, 실측 최대
	// 8000m대) 요격 구간에서도 그대로 고정 사용되고 있었음. 이 거리를 실제로
	// 좁히는 데는 보통 수십 초가 걸리는데, 1.5초짜리 예측은 사실상 "타겟의
	// 현재 위치"를 그대로 쫓는 것과 다름없어서 -- 특히 고도차가 클 때 그
	// 고도까지 다 맞춰 올라간 시점엔 이미 타겟이 다른 곳으로 이동해버림
	// (실측: 뷰어 관찰 -- 고도 맞춰 올라간 뒤 타겟을 못 찾고 다시 내려옴).
	// 실제 도달까지 걸릴 시간(거리/내 속도)만큼 미리 내다보게 스케일한다.
	// 원운동 가정이 오래 유지될수록(=너무 먼 미래일수록) 신뢰도가 떨어지므로
	// 상한(30초)을 둔다. 근접 시엔 포트 지정값(LeadTime) 밑으로는 안 내려간다.
	double EffectiveLeadTime = LeadTime;
	{
		double dist = ((*BB)->MyLocation_Cartesian - (*BB)->TargetLocaion_Cartesian).length();
		double mySpeed = (double)(*BB)->MySpeed_MS;
		if (mySpeed < 50.0) mySpeed = 50.0; // 저속/0 나눗셈 방지
		double timeToClose = dist / mySpeed;
		if (timeToClose > 30.0) timeToClose = 30.0;
		if (timeToClose > EffectiveLeadTime) EffectiveLeadTime = timeToClose;
	}

	// Predicted target position EffectiveLeadTime seconds from now, using the
	// closed-form solution for constant-turn-rate motion (same formula as
	// Task_Pure). As omega approaches 0 (straight flight) this reduces to plain
	// linear lead.
	// 2026-08-21(전술 재설계로 복원): 위 순정추적 단순화 이후에도 own_offaxis가
	// 안 좁혀지는 게 계속 확인됐고, 전체 파이프라인(데이터 매핑/투영수식/좌표
	// 변환)을 끝까지 검증해도 버그가 없었음 -- 선회율 실측(우리 1.33도/초 vs
	// 상대 1.50도/초, 거의 동률)이 원인으로 지목됨. 선회율이 비슷한 상대를
	// "꼬리(현재/예측 위치)를 쫓는" 순정추적으로는 원리적으로 안 좁혀짐(추적
	// 곡선이 각속도/속도 우위 없이는 수렴 안 함). 코너컷팅(선회 중심을
	// 조준해서 각속도 우위 없이도 기하학적으로 수렴 가능한 정석 기법)을
	// 되살린다 -- 이걸 껐던 이유(Omega 노이즈로 조준점 폭주)는 위에서 이미
	// 반경게이트(MAX_SANE_TURN_RADIUS)로 별도 해결됨.
	bool isTurning = std::fabs(Omega) >= OMEGA_EPS && (V / std::fabs(Omega)) <= MAX_SANE_TURN_RADIUS;

	double leadDx, leadDy;
	if (!isTurning)
	{
		leadDx = V * std::cos(Heading) * EffectiveLeadTime;
		leadDy = V * std::sin(Heading) * EffectiveLeadTime;
	}
	else
	{
		double futureHeading = Heading + Omega * EffectiveLeadTime;
		leadDx = (V / Omega) * (std::sin(futureHeading) - std::sin(Heading));
		leadDy = -(V / Omega) * (std::cos(futureHeading) - std::cos(Heading));
	}
	// 2026-08-20(수정): 수평 리드(leadDx/leadDy)는 원운동 가정으로 궤도 안에
	// 묶여있지만, 수직은 아무 제한 없이 "지금 자세로 EffectiveLeadTime(최대
	// 30초)만큼 계속 상승/하강"으로 선형 외삽되고 있었음 -- 실측: 상대가
	// 그 순간 살짝 피치업이었다는 이유만으로 우리가 9500m 밖에서 12600m까지
	// 치솟는 버그가 됨(상대는 6400m). 수직 자세는 몇 초 이상 잘 안 유지되니
	// 수직 리드에는 훨씬 짧은 시간(5초 캡)만 쓴다.
	double leadTimeZ = EffectiveLeadTime < 5.0 ? EffectiveLeadTime : 5.0;
	double leadDz = TargetForward.Z * V * leadTimeZ;
	Vector3 LeadPos = (*BB)->TargetLocaion_Cartesian + Vector3(leadDx, leadDy, leadDz);

	Vector3 AimPoint;

	if (!isTurning)
	{
		// Straight-flying target: no turn circle, just aim at the predicted position.
		AimPoint = LeadPos;
	}
	else
	{
		// Turn center: the time-independent part of the constant-turn-rate closed
		// form solution.
		//   x(t) = [x0 - (V/omega)sin(theta0)] + (V/omega)sin(theta0+omega t)
		//   y(t) = [y0 + (V/omega)cos(theta0)] - (V/omega)cos(theta0+omega t)
		// The bracketed part is the center; the rest traces a circle of radius
		// R = |V/omega| around it.
		double VoverOmega = V / Omega;
		double Cx = (*BB)->TargetLocaion_Cartesian.X - VoverOmega * std::sin(Heading);
		double Cy = (*BB)->TargetLocaion_Cartesian.Y + VoverOmega * std::cos(Heading);
		// 2026-08-20: 위 LeadPos.Z와 동일한 이유로, 코너컷팅용 Center도 현재
		// 고도가 아니라 EffectiveLeadTime만큼의 고도 변화 추세를 반영한다
		// (안 그러면 alpha가 커지는 원거리에서 Center만 쓰게 되는데 그게
		// 여전히 "현재 고도" 고정값이라 같은 문제가 재발함).
		double Cz = (*BB)->TargetLocaion_Cartesian.Z + leadDz;
		Vector3 Center(Cx, Cy, Cz);

		double R = std::fabs(VoverOmega);
		if (R < MIN_RADIUS) R = MIN_RADIUS;

		double d = ((*BB)->MyLocation_Cartesian - Center).length();

		// alpha=0 when we're at/inside the circle (aim at the predicted merge point),
		// alpha=1 when we're far outside (aim at the center to cut the corner),
		// linearly blended in between.
		double alpha = (d / R - 1.0) / (CutoffMult - 1.0);
		if (alpha < 0.0) alpha = 0.0;
		if (alpha > 1.0) alpha = 1.0;

		AimPoint = Center * alpha + LeadPos * (1.0 - alpha);
	}

	// 2026-08-21(신규 발견 대응): leadDz/Cz(위 고도 외삽)에 지면 하한이 없어서
	// 실측(라이브 [LOS_DBG] 990틱 분석)으로 조준점 고도(VPz)가 6%는 음수
	// (지하, 예: -626m, -10.9m)까지 튀었고, UTAngle도 48.3%가 90도 초과로
	// 요동(-172~+180도)함이 확인됨 -- 컨트롤러가 못 도는 게 아니라 도는
	// 대상(조준점) 자체가 물리적으로 말이 안 되는 곳으로 튀는 게 원인.
	// 조준점 고도를 지면 근처 하한으로 클램프해서 이 극단값만 제거한다.
	if (AimPoint.Z < 50.0) AimPoint.Z = 50.0;
	(*BB)->VP_Cartesian = AimPoint;

	// 2026-08-20(31차): 스폰 직후부터 이 Task가 무조건 100% 스로틀이라,
	// 강하가 시작되기도 전부터 계속 가속만 함(실측: 200->418m/s까지
	// 끊김없이 증가). 저고도에서 스로틀을 줄이는 시도(19차)는 이미 늦은
	// 뒤라 효과가 없었음(중력이 압도적) -- 대신 애초에 위험한 속도까지
	// 못 가게 원거리 구간부터 미리 낮춘다. 70%로 고정(추격 성능은 유지
	// 하되 무한 가속은 막음).
	(*BB)->Throttle = 0.7f;

	return NodeStatus::SUCCESS;
}
