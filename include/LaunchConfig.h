
/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 15-03-2022
 *  Author: LN

 *  config setting for Tanway LIDARs
**************************************************/

#ifndef LAUNCHCONFIG_H_
#define LAUNCHCONFIG_H_

#include <strings.h>
#include <ros/ros.h> //generic C++ stuff

class LaunchConfig
{
public:
	LaunchConfig();
	~LaunchConfig();

	void ReadLaunchParams(ros::NodeHandle& nh_private);

public:
	std::string m_localHost = "" ;
	std::string m_lidarHost = "" ;
	int m_localPort = -1;
	int m_lidarPort = -1;
	std::string m_frameID = "TanwayTP" ;
	std::string m_topic = "/tanwaylidar_pointcloud" ;

	int m_lidarType = -1;

	double m_correctedAngle1 = 0;
	double m_correctedAngle2 = 0;
	double m_correctedAngle3 = 0;

	double m_leftMoveAngle = 0;
	double m_rightMoveAngle = 0;
};

#endif
