#include "mandeye_ros/utils/TimeStampReceiver.h"
#include "mandeye_ros/utils/TimeStampProvider.h"
namespace mandeye_utils
{

void TimeStampReceiver::SetTimeStampProvider(std::shared_ptr<TimeStampProvider> timeStampProvider)
{
	m_timeStampProvider = timeStampProvider;
}

double TimeStampReceiver::GetTimeStamp()
{
	if (m_timeStampProvider)
	{
		return m_timeStampProvider->getTimestamp();
	}
	return 0.0;
}


}

