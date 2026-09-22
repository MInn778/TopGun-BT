#pragma once
#include "../../behaviortree_cpp_v3/action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	// 2026-08-16 (2차 튜닝): WEZ 근접전(Distance <= 914m) 조준 기동.
	// 순수 추적(리드 0)은 상대가 선회 중일 때 pursuit-curve lag(조준각이 일정 오프셋에서
	// 안 좁혀지는 현상, 실측 min ata=26deg)이 생겨 데미지 요건(<=1deg)을 못 맞췄다.
	// LOS(조준선) 방향 자체의 회전 속도를 매 틱 추정해서 짧게 앞서 예측하는
	// 비례항법(Proportional Navigation) 방식으로 교체 -- 이전 프레임의 LOS 방향을
	// 기억해야 하므로 노드 인스턴스에 상태를 들고 있는다(트리/노드는 전투기별로 별도
	// 인스턴스라 교차 오염 없음, CPPBehaviorTree.cpp::init() 참고).
	class Task_WEZAttack : public SyncActionNode
	{
	private:
		bool m_hasPrevLos = false;
		Vector3 m_prevLos;

	public:
		Task_WEZAttack(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~Task_WEZAttack()
		{
		}

		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
