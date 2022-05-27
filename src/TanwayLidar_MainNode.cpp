/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 15-03-2022
 *  Author: LN

 *  Node for Tanway 3D LIDARs   
**************************************************/

#include <ros/ros.h>
#include <ros/console.h>
#include "LaunchConfig.h"
#include "../sdk/TanwayLidarSDK.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h> //use these to convert between PCL and ROS datatypes
#include <pcl/conversions.h>

ros::Publisher rosPublisher;

void pointCloudCallback(TWPointCloud<pcl::PointXYZI>::Ptr twPointCloud)
{
	/*
	*The point cloud struct uses a smart pointer. 
	*Please copy the point cloud data to another thread for use.
	*Avoid directly operating the UI in the callback function.
	*/
	//std::cout << "width:" << twPointCloud->width 
	//		  << " height:" << twPointCloud->height 
	//		  << " point cloud size: " << twPointCloud->Size() << std::endl;
	
	
	//to pcl point cloud
	pcl::PointCloud<pcl::PointXYZI> cloud;
	cloud.width = twPointCloud->width;
	cloud.height = twPointCloud->height;
	cloud.header.frame_id = twPointCloud->frame_id;
	cloud.points.assign(twPointCloud->m_pointData.begin(), twPointCloud->m_pointData.end());

	//to ros point cloud
	sensor_msgs::PointCloud2 rosPointCloud; 
	pcl::toROSMsg(cloud, rosPointCloud); //convert between PCL and ROS datatypes
	rosPointCloud.header.stamp = ros::Time::now(); //Get ROS system time
	rosPublisher.publish(rosPointCloud); //Publish cloud
}

void gpsCallback(std::string gps_value)
{
	/*
	*Avoid directly operating the UI in the callback function.
	*/
	std::cout << gps_value << std::endl;
}

void exceptionCallback(const TWException& exception)
{
	/* 
	*This callback function is called when the SDK sends a tip or raises an exception problem.
	*Use another thread to respond to the exception to avoid time-consuming operations.
	*/
	if (exception.GetErrorCode() > 0)
		std::cout << "[Error Code]: " << exception.GetErrorCode() << " -> " << exception.ToString() << std::endl;
	if (exception.GetTipsCode() > 0)
		std::cout << "[Tips Code]: " << exception.GetTipsCode() << " -> " << exception.ToString() << std::endl;	
}


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "tanwaylidarview"); 
	ros::NodeHandle nh;
	//ros::Rate r(20);

	ros::NodeHandle nh_private("~");

	ROS_INFO( "tanway lidar viewer for ROS" );
	ROS_INFO( "Update Date: 2022/04/20\n" );
	ROS_INFO( "View in rviz");

	//读取Launch配置文件
	LaunchConfig launchConfig;
	launchConfig.ReadLaunchParams(nh_private);
	rosPublisher = nh.advertise<sensor_msgs::PointCloud2> (launchConfig.m_topic, 1);

	TanwayLidarSDK<pcl::PointXYZI> lidar(launchConfig.m_lidarHost, launchConfig.m_localHost, launchConfig.m_localPort, (TWLidarType)(launchConfig.m_lidarType));
	lidar.RegPointCloudCallback(pointCloudCallback);
	lidar.RegGPSCallback(gpsCallback);
	lidar.RegExceptionCallback(exceptionCallback);
	if (LT_TSP0332 == launchConfig.m_lidarType)
		lidar.SetCorrectedAngleToTSP0332(launchConfig.m_correctedAngle1, launchConfig.m_correctedAngle2);
	else if (LT_Scope192 == launchConfig.m_lidarType)
		lidar.SetCorrectedAngleToScope192(launchConfig.m_correctedAngle1, launchConfig.m_correctedAngle2, launchConfig.m_correctedAngle3);
	else if (LT_Duetto == launchConfig.m_lidarType)
		lidar.SetMoveAngleToDuetto(launchConfig.m_leftMoveAngle, launchConfig.m_rightMoveAngle);
	
	lidar.Start();

	while (ros::ok())
	{
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	return 0;
}
