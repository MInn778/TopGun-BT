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
	// 2026-08-17: 기존 DECO_AltitudeCheck는 매 틱 현재 고도만 보고 판정해서,
	// TriggerAltitude(예: 650m)를 넘는 순간 바로 추격 모드로 복귀 -> 여유 고도를
	// 충분히 못 벌고 곧바로 다음 위기가 반복되는 패턴이 실측으로 확인됨(58초에
	// 7번 발동). 이 노드는 "한 번 위기(TriggerAltitude 밑)에 들어가면,
	// ReleaseAltitude(더 높은 값)를 넘을 때까지는 계속 위기 상태로 유지"하는
	// 이력(hysteresis)을 노드 인스턴스에 기억해둔다. 노드/트리는 전투기별로
	// 별도 인스턴스라 교차 오염 없음(CPPBehaviorTree.cpp::init() 참고).
	class DECO_EmergencyGuard : public SyncActionNode
	{
	private:
		bool m_inRecovery = false;

	public:
		DECO_EmergencyGuard(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~DECO_EmergencyGuard()
		{
		}

		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
