#pragma once
#include "../../behaviortree_cpp_v3/action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	// 2026-08-16: 원거리 접근(Distance > 3000m) 구간. 완전 리드(intercept) 조준점으로
	// 최단 경로 요격 침로를 잡아 거리를 빠르게 좁힌다.
	class Task_LongRangeApproach : public SyncActionNode
	{
	public:
		Task_LongRangeApproach(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~Task_LongRangeApproach()
		{
		}

		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
