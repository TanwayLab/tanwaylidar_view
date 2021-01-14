#include <LaunchConfig.h>

LaunchConfig::LaunchConfig()
{

}

LaunchConfig::~LaunchConfig()
{

}
void LaunchConfig::ReadLaunchParams(ros::NodeHandle& nh_private)
{
	nh_private.param<std::string>("host", m_host, "192.168.111.204");
	nh_private.param<std::string>("LiDARhost", m_lidarhost, "192.168.111.51");
	nh_private.param<std::string>("frame_id", m_frameID, "TanwayTP");
	nh_private.param<std::string>("topic", m_topic, "/tensorpro_cloud");
	nh_private.param<int>("port", m_localPort, 5600);
	nh_private.param<int>("LiDARport", m_lidarPort, 5051);
	nh_private.param<bool>("timestamp_print_switch", m_timestampPrintSwitch, false);
	nh_private.param<double>("StartAngle", m_startAngle, 30.0);
	nh_private.param<double>("EndAngle", m_endAngle, 150.0);

	int lidarType = 1;
	nh_private.param<int>("LidarType", lidarType, 1);
	m_lidarType = (LidarType)lidarType;

}

