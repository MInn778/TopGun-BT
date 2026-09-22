#pragma once
#include "../../behaviortree_cpp_v3/action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	// 2026-08-16: 중거리 추격(914m < Distance <= 3000m) 구간. 절반 리드 펄슛으로
	// 선회를 잘라들어가(cut the circle) AA를 줄이면서 WEZ 진입을 준비한다.
	//
	// 2026-08-20(신규): "6시 방향" 조준점이 상대의 "지금" 위치+진행방향으로만
	// 계산돼서, 상대가 선회 중이면 조준점도 계속 따라 회전하며 도망감 --
	// Task_Intercept.h 헤더 주석의 문제의식과 동일("chasing a point on the
	// EDGE of the target's turn circle keeps pushing the pursuer wide").
	// 같은 등속 원운동 추정(Task_Intercept와 동일 방식)을 여기도 적용해서,
	// "지금 꼬리"가 아니라 "선회를 감안했을 때의 미래 꼬리"를 조준한다.
	class Task_MidRangeEngage : public SyncActionNode
	{
	private:
		bool   m_HasPrevHeading = false;
		double m_PrevHeading = 0.0;
		double m_PrevRunningTime = 0.0;
		double m_OmegaFiltered = 0.0;	// estimated turn rate (rad/s), exponentially smoothed

	public:
		Task_MidRangeEngage(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~Task_MidRangeEngage()
		{
		}

		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
