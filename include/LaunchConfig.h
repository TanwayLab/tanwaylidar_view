
/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 30-01-2021
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

	enum LidarType
	{
		LT_Tensor_Lite, //0
		LT_Tensor_Pro,	//1
		LT_Scope		//2
	};

	void ReadLaunchParams(ros::NodeHandle& nh_private);

public:
	std::string m_host = "" ;
	std::string m_lidarhost = "" ;
	std::string m_frameID = "TanwayTP" ;
	std::string m_topic = "/tanwaylidar_pointcloud" ;	
	int m_localPort = -1;
	int m_lidarPort = -1;	
	double m_startAngle = 30;
	double m_endAngle = 150;

	LidarType m_lidarType;
};

#endif