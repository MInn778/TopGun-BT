#include "Task_WEZAttack.h"

namespace Action
{
	PortsList Task_WEZAttack::providedPorts()
	{
		return {
				InputPort<CPPBlackBoard*>("BB")
		};
	}

	NodeStatus Task_WEZAttack::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		Vector3 toTarget = (*BB)->TargetLocaion_Cartesian - (*BB)->MyLocation_Cartesian;
		double dist = toTarget.length();
		Vector3 losNow = (dist > 1e-3) ? (toTarget / dist) : Vector3(1.0, 0.0, 0.0);

		double dt = (*BB)->DeltaSecond;
		Vector3 aimDir = losNow;

		if (m_hasPrevLos && dt > 1e-4)
		{
			// LOS(조준선) 방향 벡터의 초당 변화율을 각속도로 근사하고, 그 회전을
			// 앞서 예측한 방향을 조준점으로 삼는다. 순수 추적(리드 0)은
			// 상대가 선회 중일 때 pursuit-curve lag으로 조준각이 일정 오프셋에서
			// 안 좁혀지는 문제(실측: 최소 own_ata 26도)가 있었는데, 이 방식은
			// 상대의 회전을 따라가며 그 오프셋을 줄이는 걸 목표로 한다.
			//
			// 2026-08-17 (좌표 버그 수정 후 재튜닝): 고정 0.35초 리드로는 헤드온
			// 머지처럼 짧게 스쳐지나가는 패스에서 13도 정도까지밖에 안 좁혀짐
			// (실측). 아직 많이 안 맞았을 때(Los_Degree 큼)는 리드를 더 크게 줘서
			// 상대 선회를 더 적극적으로 따라잡고, 거의 다 맞았을 때(Los_Degree 작음)는
			// 리드를 줄여 오버슛 없이 정밀 추종하도록 Los_Degree에 비례해 조절.
			double losDeg = (*BB)->Los_Degree;
			double lookaheadSec = losDeg / 30.0;
			lookaheadSec = lookaheadSec < 0.2 ? 0.2 : (lookaheadSec > 1.2 ? 1.2 : lookaheadSec);

			// 2026-08-22 (v2에서 포팅): 근접 통과(머지) 구간에서는 LOS가 한 프레임
			// 만에 거의 반대 방향으로 스윕하면서 losRate가 폭발적으로 커지고, 그
			// 결과 predicted 조준점이 요동쳐 기체가 그걸 쫓아 급기동 -> 에너지
			// 소진 -> 회전 못 이기고 텀블/실속으로 추락하는 사례가 v2에서 실측
			// 확인됨. v1도 이번 라이브(v1_live_log.txt)에서 192m/196m까지 두 번
			// 근접 통과가 있었던 게 확인돼 같은 위험 구간에 노출돼 있음 -- 이
			// 거리대는 리드를 걸 시간 여유도 없으므로 순수 추적(리드 0)으로
			// 강제해 폭주를 원천 차단한다.
			if (dist < 300.0)
			{
				aimDir = losNow;
			}
			else
			{
				Vector3 losRate = (losNow - m_prevLos) / dt;
				Vector3 predicted = losNow + losRate * lookaheadSec;
				double predictedLen = predicted.length();
				if (predictedLen > 1e-3)
				{
					aimDir = predicted / predictedLen;
				}
			}
		}

		m_prevLos = losNow;
		m_hasPrevLos = true;

		(*BB)->VP_Cartesian = (*BB)->MyLocation_Cartesian + aimDir * 1000.0;

		// 2026-08-22 (v2에서 포팅): WEZ(914m) 안에서 거리 무관하게 항상 풀스로틀로
		// 직진 추적만 하면 체류시간 없이 관통(blow-through)하는 문제가 v2에서
		// 있었음. 가까울수록 스로틀을 줄여 관통 속도를 낮춘다(914m=100%->
		// 300m 이하=50%, 선형). 단, v1의 이번 라이브 실측(WEZ 진입 6회, 각
		// 3~10초 체류, 192m/196m까지 근접)으로는 관통 자체는 이미 문제가 아닌
		// 것으로 보여 -- 그래도 크래시 방지(위 dist<300 분기)와 세트로 안전하게
		// 같이 이식.
		double throttleFrac = (dist - 300.0) / (914.0 - 300.0);
		if (throttleFrac < 0.0) throttleFrac = 0.0;
		if (throttleFrac > 1.0) throttleFrac = 1.0;
		(*BB)->Throttle = (float)(0.5 + 0.5 * throttleFrac);

		return NodeStatus::SUCCESS;
	}
}
