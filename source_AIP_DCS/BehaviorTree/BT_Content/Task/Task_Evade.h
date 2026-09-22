#pragma once
#include "../../behaviortree_cpp_v3/action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	// 2026-08-17: 상대가 나를 조준 중일 때(DECO_ThreatCheck) 실행되는 방어 기동.
	// 상대 조준선에서 최대한 빨리 벗어나도록 옆으로 급선회(break turn) + 수직 분리.
	class Task_Evade : public SyncActionNode
	{
	public:
		Task_Evade(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~Task_Evade()
		{
		}

		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
