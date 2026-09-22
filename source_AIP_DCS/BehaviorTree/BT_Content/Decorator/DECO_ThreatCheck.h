#pragma once
#include "../../behaviortree_cpp_v3/action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include <iostream>
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	// 2026-08-17: 상대가 나를 얼마나 잘 조준하고 있는지(Los_Degree_Target, CheckSight.cpp가
	// 매 틱 계산)를 기준으로 위협 여부를 판단하는 조건 노드. DECO_LOSCheck(내가 상대를
	// 조준하는 정도)의 반대 방향 버전.
	class DECO_ThreatCheck : public SyncActionNode
	{
	public:
		DECO_ThreatCheck(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~DECO_ThreatCheck()
		{
		}

		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
