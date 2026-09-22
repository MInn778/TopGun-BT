#include "DECO_EmergencyGuard.h"

namespace Action
{
	PortsList DECO_EmergencyGuard::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<std::string>("TriggerAltitude"),
			InputPort<std::string>("ReleaseAltitude")
		};
	}

	NodeStatus DECO_EmergencyGuard::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		Optional<std::string> triggerStr = getInput<std::string>("TriggerAltitude");
		Optional<std::string> releaseStr = getInput<std::string>("ReleaseAltitude");

		double triggerAltitude = std::stod(triggerStr.value());
		double releaseAltitude = std::stod(releaseStr.value());
		double currentAltitude = (*BB)->MyLocation_Cartesian.Z;

		if (!m_inRecovery && currentAltitude < triggerAltitude)
		{
			m_inRecovery = true;
		}
		else if (m_inRecovery && currentAltitude >= releaseAltitude)
		{
			m_inRecovery = false;
		}

		return m_inRecovery ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
	}
}
