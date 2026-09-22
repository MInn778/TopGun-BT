#include "Task_Evade.h"

namespace Action
{
	PortsList Task_Evade::providedPorts()
	{
		return {
				InputPort<CPPBlackBoard*>("BB")
		};
	}

	NodeStatus Task_Evade::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		// 상대에게서 멀어지는 방향(거리 벌기) + 오른쪽/위쪽으로 급선회(조준선 이탈)를
		// 섞은 조준점. 실제 브레이크 턴처럼 "상대 총선에서 최대한 빨리 벗어나기"를
		// 노림 -- 어느 쪽이 더 유리한지 판단하는 로직은 없이 항상 같은 방향(오른쪽+위)
		// 으로 도는 단순한 첫 버전.
		Vector3 toThreat = (*BB)->TargetLocaion_Cartesian - (*BB)->MyLocation_Cartesian;
		double dist = toThreat.length();
		Vector3 awayDir = (dist > 1e-3) ? (toThreat / dist) * -1.0 : Vector3(-1.0, 0.0, 0.0);

		(*BB)->VP_Cartesian = (*BB)->MyLocation_Cartesian
			+ (*BB)->MyRightVector * 2000.0
			+ Vector3(0.0, 0.0, 1200.0)
			+ awayDir * 500.0;

		(*BB)->Throttle = 1.0f;

		return NodeStatus::SUCCESS;
	}
}
