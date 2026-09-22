#include "Task_LongRangeApproach.h"

namespace Action
{
	PortsList Task_LongRangeApproach::providedPorts()
	{
		return {
				InputPort<CPPBlackBoard*>("BB")
		};
	}

	NodeStatus Task_LongRangeApproach::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		// 2026-08-16 (4차 튜닝, 실측으로 발견한 진짜 근본원인 수정):
		// closure-time 전체(먼거리에서는 40~50초!)를 리드로 쓰면 "상대가 지금 방향으로
		// 50초간 직진한다"고 가정하는 셈인데, 실제로는 상대도 계속 선회한다 --
		// 미러전 실측 결과 초반 1회 교차(merge) 후 남은 200~300초 내내 8000~17000m
		// 밖에서 다시는 못 붙는 patttern이 나왔다(양쪽 다 서로의 틀린 예측점을
		// 쫓아가며 수렴 안 하는 궤도를 만든 것으로 추정). 리드를 짧게(최대 3초) 캡
		// 씌워서 순수 추적에 가깝게 만들면, 상대 속도가 같아도(mirror) 거리가
		// 단조 감소하는 안정적인 pursuit curve로 수렴한다.
		float rawLeadTime = (float)((*BB)->Distance / ((*BB)->MySpeed_MS + 1.0));
		float leadTime = rawLeadTime < 3.0f ? rawLeadTime : 3.0f;

		Vector3 targetVelocity = (*BB)->TargetForwardVector * (*BB)->TargetSpeed_MS;
		Vector3 aimPoint = (*BB)->TargetLocaion_Cartesian + targetVelocity * leadTime;

		// 2026-08-16 (5차 튜닝): 리드 캡만으로는 정면 충돌항로(dead head-on merge)
		// 자체는 안 없어져서, merge 이후 180도 완전반전에 계속 시간이 걸렸다
		// (실측: 200초 내 2차 WEZ 진입 실패, 3000~7000m대에서 진동만 반복).
		// 실제 공중전의 "lead-turn"처럼, 원거리에서 내 오른쪽으로 미리 살짝 비켜
		// 조준해서 정면충돌이 아니라 한쪽으로 스쳐지나가는 항로를 만든다 --
		// 그러면 merge 직후 필요한 회두각이 180도가 아니라 훨씬 작아진다.
		// 양쪽 다 "내 오른쪽"으로 비키므로(대칭), 서로 반대편으로 갈라지지 않고
		// 같은 회전 방향으로 스쳐지나가 선회전으로 자연스럽게 이어지는 걸 노림.
		double lateralBias = (*BB)->Distance * 0.6;
		lateralBias = lateralBias < 1500.0 ? 1500.0 : (lateralBias > 6000.0 ? 6000.0 : lateralBias);
		aimPoint = aimPoint + (*BB)->MyRightVector * lateralBias;

		// 2026-08-17: 고도유지 보정 두 강도 다 60개 시나리오 배치 검증에서 개선
		// 효과 없어서(Task_MidRangeEngage 주석 참고) 제거, 원래 버전으로 되돌림.
		(*BB)->VP_Cartesian = aimPoint;

		(*BB)->Throttle = 1.0f;

		return NodeStatus::SUCCESS;
	}
}
