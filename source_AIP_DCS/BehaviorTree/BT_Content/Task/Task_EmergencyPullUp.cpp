#include "Task_EmergencyPullUp.h"

PortsList Action::Task_EmergencyPullUp::providedPorts()
{
	return {
		InputPort<CPPBlackBoard*>("BB")
	};
}

NodeStatus Action::Task_EmergencyPullUp::tick()
{
	Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

	// 2026-08-16 튜닝: 저고도 미러전에서 900m 트리거로도 지면 충돌(298m)이 발생해
	// 회피각을 더 가파르게(atan(1500/2000)=37deg -> atan(3000/800)=75deg 근처)
	// 올리고, 트리거 고도도 XML에서 450->650m로 앞당김(반응 여유 확보).
	//
	// 2026-08-17: alpha=90/seed=1 단일 사례의 회복 실패(스핀성 이탈)를 고치려고
	// forward 의존도를 낮추고 트리거를 900m로 올려봤으나, 6개 시나리오 전체로
	// 재검증하니 오히려 승률이 5W/1L -> 2W/1L/3D로 악화됨(회피 모드에 더 자주/오래
	// 머물면서 공격 기회 자체가 줄어듦, 무교전 무승부 3건 발생). 단일 실패 사례에
	// 과최적화한 것으로 판단해 이 버전(800/3000, 트리거 650m)으로 되돌림.
	// alpha=90/seed=1의 저고도 자멸 리스크는 알려진 한계로 남겨둠.
	(*BB)->VP_Cartesian = (*BB)->MyLocation_Cartesian + (*BB)->MyForwardVector * 800.0f + Vector3(0.0f, 0.0f, 3000.0f);

	// Apply maximum throttle to gain energy/speed during climbing
	(*BB)->Throttle = 1.0f;

	return NodeStatus::SUCCESS;
}
