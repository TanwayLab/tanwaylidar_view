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
#include <sensor_msgs/Imu.h>
#include <pcl/conversions.h>

ros::Publisher rosPublisher; 
ros::Publisher rosIMUPublisher; 

struct TanwayPCLEXPoint
{
  PCL_ADD_POINT4D;

	float intensity;
  	int channel;
	float angle;
	int echo;
	int block;				/*For duetto*/
  	unsigned int t_sec;     /* The value represents seconds since 1900-01-01 00:00:00 (the UNIX epoch).*/ 
	unsigned int t_usec;    /* remaining microseconds */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(TanwayPCLEXPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (int, channel, channel)
                                  (float, angle, angle)	
                                  (int, echo, echo)
								  (int, block, block)
								  (unsigned int, t_sec, t_sec)
								  (unsigned int, t_usec, t_usec)
                                 )


void pointCloudCallback(TWPointCloud<TanwayPCLEXPoint>::Ptr twPointCloud, bool lostPacket)
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
	pcl::PointCloud<TanwayPCLEXPoint> cloud;
	cloud.width = twPointCloud->width;
	cloud.height = twPointCloud->height;
	cloud.header.frame_id = twPointCloud->frame_id;
	cloud.header.stamp = twPointCloud->stamp;
	cloud.points.assign(twPointCloud->m_pointData.begin(), twPointCloud->m_pointData.end());

	//to ros point cloud
	sensor_msgs::PointCloud2 rosPointCloud;
	pcl::toROSMsg(cloud, rosPointCloud); //convert between PCL and ROS datatypes
	rosPublisher.publish(rosPointCloud); //Publish point cloud
}

void imuCallback(const TWIMUData& imu)
{
	/*
	*Avoid directly operating the UI in the callback function.
	*/
	sensor_msgs::Imu imu_data;
	uint32_t sec = imu.stamp/1000000;
	uint32_t nsec = (imu.stamp - sec*1000000) * 1000;
	imu_data.header.stamp =ros::Time(sec, nsec) ;
	imu_data.header.frame_id = imu.frame_id;

	imu_data.linear_acceleration.x = imu.linear_acceleration[0]; 
	imu_data.linear_acceleration.y = imu.linear_acceleration[1];
	imu_data.linear_acceleration.z = imu.linear_acceleration[2];

	imu_data.angular_velocity.x = imu.angular_velocity[0]; 
	imu_data.angular_velocity.y = imu.angular_velocity[1]; 
	imu_data.angular_velocity.z = imu.angular_velocity[2];

	rosIMUPublisher.publish(imu_data); //Publish IMU
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

	ROS_INFO( "tanway lidar viewer for ROS1");
	ROS_INFO( "Version 3.0.10, 2024-04-09.");
	ROS_INFO( "View in rviz");

	//读取Launch配置文件
	LaunchConfig launchConfig;
	launchConfig.ReadLaunchParams(nh_private);
	rosPublisher = nh.advertise<sensor_msgs::PointCloud2> (launchConfig.m_topic, 1);
	rosIMUPublisher = nh.advertise<sensor_msgs::Imu> (launchConfig.m_imuTopic, 1);
	
	//on-line
	if ("on-line" == launchConfig.m_connectType)
	{
		TanwayLidarSDK<TanwayPCLEXPoint> lidar(launchConfig.m_lidarHost, launchConfig.m_localHost, launchConfig.m_localPointCloudPort, launchConfig.m_localDIFPort, (TWLidarType)(launchConfig.m_lidarType));
		lidar.RegPointCloudCallback(pointCloudCallback);
		lidar.RegIMUDataCallback(imuCallback);
		lidar.RegGPSCallback(gpsCallback);
		lidar.RegExceptionCallback(exceptionCallback);
		lidar.SetTransform(
			launchConfig.m_transformRotateX, 
			launchConfig.m_transformRotateY, 
			launchConfig.m_transformRotateZ,
			launchConfig.m_transformMoveX,
			launchConfig.m_transformMoveY,
			launchConfig.m_transformMoveZ
		);
		
		if (LT_TSP0332 == launchConfig.m_lidarType)
			lidar.SetCorrectedAngleToTSP0332(launchConfig.m_correctedAngle1, launchConfig.m_correctedAngle2);
		else if (LT_Scope192 == launchConfig.m_lidarType)
			lidar.SetCorrectedAngleToScope192(launchConfig.m_correctedAngle1, launchConfig.m_correctedAngle2, launchConfig.m_correctedAngle3);
		else if (LT_ScopeMiniA2_192 == launchConfig.m_lidarType || LT_TempoA2 == launchConfig.m_lidarType)
			lidar.SetCorrectionAngleToScopeMiniA2_192(launchConfig.m_correctedAngle1, launchConfig.m_correctedAngle2, launchConfig.m_correctedAngle3);
		else if (LT_Focus == launchConfig.m_lidarType) 
		{
			lidar.SetCorrectionAngleToScopeMiniA2_192(launchConfig.m_correctedAngle1, launchConfig.m_correctedAngle2, launchConfig.m_correctedAngle3);
			lidar.SetJointabc(launchConfig.m_bJointabc, launchConfig.m_jointabc_node1,launchConfig.m_jointabc_node2, launchConfig.m_jointabc_one_face, launchConfig.m_jointabc_two_face); 
		}

		lidar.Start();

		while (ros::ok())
		{
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}

		return 0;
	}
	//off-line
	else
	{
		TanwayLidarSDK<TanwayPCLEXPoint> lidar(launchConfig.m_filePath, launchConfig.m_lidarHost, launchConfig.m_localPointCloudPort, launchConfig.m_localDIFPort, (TWLidarType)(launchConfig.m_lidarType), true);
		lidar.RegPointCloudCallback(pointCloudCallback);
		lidar.RegIMUDataCallback(imuCallback);
		lidar.RegGPSCallback(gpsCallback);
		lidar.RegExceptionCallback(exceptionCallback);
		lidar.SetTransform(
			launchConfig.m_transformRotateX, 
			launchConfig.m_transformRotateY, 
			launchConfig.m_transformRotateZ,
			launchConfig.m_transformMoveX,
			launchConfig.m_transformMoveY,
			launchConfig.m_transformMoveZ
		);
		if (LT_TSP0332 == launchConfig.m_lidarType)
			lidar.SetCorrectedAngleToTSP0332(launchConfig.m_correctedAngle1, launchConfig.m_correctedAngle2);
		else if (LT_Scope192 == launchConfig.m_lidarType)
			lidar.SetCorrectedAngleToScope192(launchConfig.m_correctedAngle1, launchConfig.m_correctedAngle2, launchConfig.m_correctedAngle3);
		else if (LT_ScopeMiniA2_192 == launchConfig.m_lidarType || LT_TempoA2 == launchConfig.m_lidarType)
			lidar.SetCorrectionAngleToScopeMiniA2_192(launchConfig.m_correctedAngle1, launchConfig.m_correctedAngle2, launchConfig.m_correctedAngle3);
		else if (LT_Focus == launchConfig.m_lidarType) 
		{
			lidar.SetCorrectionAngleToScopeMiniA2_192(launchConfig.m_correctedAngle1, launchConfig.m_correctedAngle2, launchConfig.m_correctedAngle3);
			lidar.SetJointabc(launchConfig.m_bJointabc, launchConfig.m_jointabc_node1,launchConfig.m_jointabc_node2, launchConfig.m_jointabc_one_face, launchConfig.m_jointabc_two_face); 
		}


		lidar.Start();

		while (ros::ok())
		{
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}

		return 0;
	}
}
