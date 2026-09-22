#include "DECO_AltitudeCheck.h"

namespace Action
{
	PortsList DECO_AltitudeCheck::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<std::string>("UpDown"),
			InputPort<std::string>("Altitude")
		};
	}

	NodeStatus DECO_AltitudeCheck::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		Optional<std::string> UpOrDown = getInput<std::string>("UpDown");
		Optional<std::string> AltVal = getInput<std::string>("Altitude");

		// MyLocation_Cartesian.Z is altitude in meters.
		float CurrentAltitude = (*BB)->MyLocation_Cartesian.Z;
		std::string UD = UpOrDown.value();
		float InputAltitude = std::stof(AltVal.value());

		if (UD == "Greater")
		{
			if (CurrentAltitude >= InputAltitude)
			{
				return NodeStatus::SUCCESS;
			}
			else
				return NodeStatus::FAILURE;
		}
		else if (UD == "Less")
		{
			if (CurrentAltitude <= InputAltitude)
			{
				return NodeStatus::SUCCESS;
			}
			else
				return NodeStatus::FAILURE;
		}
		else
		{
			return NodeStatus::FAILURE;
		}
	}
}
