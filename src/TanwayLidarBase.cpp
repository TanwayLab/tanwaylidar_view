#include <sensor_msgs/PointCloud2.h> //ROS message type to publish a pointCloud
#include <TanwayLidarBase.h>

TanwayLidarBase::TanwayLidarBase()
{

}

TanwayLidarBase::~TanwayLidarBase()
{

}


bool TanwayLidarBase::Initialize(ros::NodeHandle& nh, LaunchConfig& config)
{

	m_rosPublisher = nh.advertise<sensor_msgs::PointCloud2> (config.m_topic, 1);
	m_startAngle = config.m_startAngle;
	m_endAngle = config.m_endAngle;

	//初始化UDP
	return m_UDPNetwork.Init(config.m_host, config.m_localPort, config.m_lidarhost, config.m_lidarPort);

}

bool TanwayLidarBase::GetUDP()
{
	m_lengthBuf = m_UDPNetwork.recvUDP(m_buf);

	if (m_lengthBuf < 0) //Recieve UDP packets
	{
		return false;
	}
	
	AnalysisUDPData();

	return true;
}

void TanwayLidarBase::SetBufferAndAnalysis(u_char* data, int len)
{
	memcpy(m_buf, data, len);

	m_lengthBuf = len;

	AnalysisUDPData();
}