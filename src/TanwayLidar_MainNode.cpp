/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 30-01-2021
 *  Author: LN

 *  Node for Tanway 3D LIDARs   
**************************************************/

#include <ros/ros.h>
#include <ros/console.h>
#include "LaunchConfig.h"
#include "ScopeView.h"
#include "TensorProView.h"



int main(int argc, char** argv) 
{
	ros::init(argc, argv, "tanwaylidarview"); 
	ros::NodeHandle nh;
	//ros::Rate r(20);

	ros::NodeHandle nh_private("~");

	ROS_INFO( "tanway lidar viewer for ROS" );
	ROS_INFO( "Version 1.0.1" );
	ROS_INFO( "Update Date: 2021/02/01\n" );
	ROS_INFO( "View in rviz;");

	//读取Launch配置文件
	LaunchConfig launchConfig;
	launchConfig.ReadLaunchParams(nh_private);

	//按设备类型，处理设备数据
	TanwayLidarBase* pLidarBase = NULL;
	switch (launchConfig.m_lidarType)
	{
		case LaunchConfig::LT_Tensor_Lite:
		case LaunchConfig::LT_Tensor_Pro:
			ROS_INFO( "[Run Info]--: Run Lidar Type: Tensor_Pro\n");
			pLidarBase = new TensorProView();
			break;
		
		case LaunchConfig::LT_Scope:
			ROS_INFO( "[Run Info]--: Run Lidar Type: Scope(Beta)\n");
			pLidarBase = new ScopeView();
			break;

		default:
			ROS_INFO( "[Run Info]--: Run Lidar Type: Unknown(error)\n");
			ros::shutdown();
			return 0;
	}

	if (!pLidarBase->Initialize(nh, launchConfig))
	{
		ROS_INFO( "[error] --Init UDP Socket Failed!\n");
		delete pLidarBase;
		pLidarBase = NULL;
		ros::shutdown();
		return 0;
	}

	while (ros::ok())
	{
		pLidarBase->GetUDP();
	}

	delete pLidarBase;
	pLidarBase = NULL;
	return 0;
}
