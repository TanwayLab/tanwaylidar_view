
#ifndef LAUNCHCONFIG_H_
#define LAUNCHCONFIG_H_

#include <strings.h>
#include <ros/ros.h> //generic C++ stuff

class LaunchConfig
{
public:
	LaunchConfig();
	~LaunchConfig();

	enum LidarType
	{
		LT_Tensor_Lite, //0
		LT_Tensor_Pro,	//1
		LT_Scope		//2
	};

	void ReadLaunchParams(ros::NodeHandle& nh_private);

public:
	std::string m_host = "192.168.111.204" ;
	std::string m_lidarhost = "192.168.111.51" ;
	std::string m_frameID = "TanwayTP" ;
	std::string m_topic = "/tensorpro_cloud" ;	
	int m_localPort = 5600;
	int m_lidarPort = 5050;	
	double m_startAngle = 30;
	double m_endAngle = 150;
	bool m_timestampPrintSwitch = false;

	LidarType m_lidarType;
};

#endif