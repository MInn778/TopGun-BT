#pragma once
/*
	teamKangnam : Turn-Circle Cutoff Intercept Task Node (2026-08-17)

	Background: Task_Pure (aim at current/predicted position) and lead pursuit (both linear
	and curved) all fail to produce a re-engagement against a target that keeps turning
	(loiter opponent) - see TopGunReadMe.md BT track 7-39/7-43/7-47/7-52. No matter how
	well the aim point predicts the target's future position, chasing a point that sits on
	the EDGE of the target's turn circle keeps pushing the pursuer wide.

	This node takes a different approach - it estimates the circle the target is currently
	flying (center + radius) in real time, and:
	  - When far from that circle (e.g. beyond 3x radius): aim at the CENTER of the circle,
	    cutting straight across instead of chasing along the rim.
	  - When close to the circle (near 1x radius): aim at the target's predicted position
	    (LeadTime seconds ahead) to actually merge.
	  - In between, blend linearly by distance.

	The turn rate (omega) is measured from tick-to-tick heading change, same as
	Task_Pure/Task_Lag - not guessed from bank angle, since a wrong sign guess would aim
	the wrong way and make things worse (see Task_Pure.h).
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	class Task_Intercept : public SyncActionNode
	{
	private:
		bool   m_HasPrevHeading = false;
		double m_PrevHeading = 0.0;
		double m_PrevRunningTime = 0.0;
		double m_OmegaFiltered = 0.0;	// estimated turn rate (rad/s), exponentially smoothed

	public:
		Task_Intercept(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~Task_Intercept()
		{
		}

		static PortsList providedPorts();

		NodeStatus tick() override;
	};
}
