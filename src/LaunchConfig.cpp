/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 30-01-2021
 *  Author: LN

 *  config setting for Tanway LIDARs
**************************************************/
#include <LaunchConfig.h>

LaunchConfig::LaunchConfig()
{

}

LaunchConfig::~LaunchConfig()
{

}
void LaunchConfig::ReadLaunchParams(ros::NodeHandle& nh_private)
{
	nh_private.param<std::string>("LocalHost", m_localHost, "192.168.111.204");
	nh_private.param<int>("LocalPort", m_localPort, 5600);

	nh_private.param<std::string>("LidarHost", m_lidarHost, "192.168.111.51");
	nh_private.param<int>("LidarPort", m_lidarPort, 5050);

	nh_private.param<std::string>("frame_id", m_frameID, "TanwayTP");
	nh_private.param<std::string>("topic", m_topic, "/tanwaylidar_pointcloud");

	nh_private.param<int>("LidarType", m_lidarType, -1);

	//TSP03-32
	if (4 == m_lidarType)
	{
		nh_private.param<double>("CorrectedAngle1", m_correctedAngle1, 0);
		nh_private.param<double>("CorrectedAngle2", m_correctedAngle2, -6.0);
	}
	//Scope-192
	else if (5 == m_lidarType)
	{
		nh_private.param<double>("CorrectedAngle1", m_correctedAngle1, 0);
		nh_private.param<double>("CorrectedAngle2", m_correctedAngle2, 0.12);
		nh_private.param<double>("CorrectedAngle3", m_correctedAngle3, 0.24);
	}
	else if (6 == m_lidarType)
	{
		nh_private.param<double>("LeftMoveAngle", m_leftMoveAngle, 30.0);
		nh_private.param<double>("RightMoveAngle", m_rightMoveAngle, 30.0);
	}
	
	
}

