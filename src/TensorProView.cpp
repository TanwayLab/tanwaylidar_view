/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-07-2019
 *  Edited on: 31-05-2020
 *  Author: Elodie Shan
 *  Editor: LF Shen

 *  Node for Tanway Tensor 3D LIDARs   
 *  Function: 20lines one bag-->switch:cout flag state
**************************************************/

#include <ros/ros.h> //generic C++ stuff
#include <pcl_ros/point_cloud.h> //use these to convert between PCL and ROS datatypes
#include <pcl/conversions.h>
#include <math.h>
#include "RawData_Utils.h"
#include "TensorProView.h"


TensorProView::TensorProView()
{
	TanwayViewPointCloud cloud;
	m_tanwayViewPointCloud = cloud.makeShared();
}
TensorProView::~TensorProView(){};

void TensorProView::AnalysisUDPData()
{
	//确定数据包长度正确
	if (m_lengthBuf != 1440) return;

	int blocks_num = 0;
	while (blocks_num < 20) 
	{
		int offset = blocks_num * 72;

		if (m_timestampPrintSwitch && blocks_num == 19)
			PrintTimeStamps(offset);

		//水平角度
		int hextoAngle = FourHexToInt(m_buf[offset+64],m_buf[offset+65],m_buf[offset+66],m_buf[offset+67]);
		float horAngle = hextoAngle/100000.0f;
		
		//按照角度范围
		if(horAngle>=m_startAngle && horAngle<=m_endAngle)
		{
			//重置分帧标志
			m_needPublishCloud = true;

			//计算临时变量值
			m_cos_hA_RA = cos(horAngle * RA);
			m_sin_hA_RA = sin(horAngle * RA);

			int seq = 0;
			while (seq < 16) 
			{
				//距离
				float hexToInt = TwoHextoInt(m_buf[offset+seq*4], m_buf[offset+seq*4+1]); 
				
				//脉宽
				float hexPulseWidth = TwoHextoInt(m_buf[offset+seq*4+2], m_buf[offset+seq*4+3]);
				
				//通道 1-16
				int channel = seq+1;

				//计算点数据
				UseAnalysisPoint(horAngle, hexToInt, channel, hexPulseWidth, offset);

				seq++;
			}

		}
		//else
		//	ROS_DEBUG("hA = [%f]",horAngle);

		if (horAngle < m_startAngle && m_needPublishCloud)
		{
			PublishCloud();
			m_needPublishCloud = false;
		}

		blocks_num++;
	}

}

void TensorProView::UseAnalysisPoint(float horAngle, float hexL, int channel, float hexPulseWidth, int offset)
{

 	//float cos_hA_RA = cos(horAngle * RA);
	//float sin_hA_RA = sin(horAngle * RA);
	float vA = verticalChannels[channel-1];
	float cos_vA_RA = cos(vA * RA);
	float L = hexL*m_tmpCal;
	float pulseWidth = hexPulseWidth*m_tmpCal;

	//距离过滤
	if (L<=0 || L > 200)  return;
	
	//创建点
	TanwayViewPoint point;
	point.x = L * cos_vA_RA * m_cos_hA_RA;
	point.y = L * cos_vA_RA * m_sin_hA_RA;
	point.z = L * sin(vA * RA);
	point.pulsewidth = pulseWidth;

	m_tanwayViewPointCloud->points.push_back(point);
}

void TensorProView::PublishCloud()
{
	//更新点云帧基础数据	
	m_tanwayViewPointCloud->width = (int) m_tanwayViewPointCloud->points.size(); //Number of points in one frame
	m_tanwayViewPointCloud->height = 1; // Whether the point cloud is orderly, 1 is disordered
	m_tanwayViewPointCloud->header.frame_id = "TanwayTP"; //Point cloud coordinate system name
	ROS_DEBUG( "Publish   num: [%d]",(int) m_tanwayViewPointCloud->points.size());

  
	pcl::toROSMsg(*m_tanwayViewPointCloud, m_rosPointCloud); //convert between PCL and ROS datatypes
	m_rosPointCloud.header.stamp = ros::Time::now(); //Get ROS system time
	m_rosPublisher.publish(m_rosPointCloud); //Publish cloud
	

	TanwayViewPointCloud cloud;
	m_tanwayViewPointCloud = cloud.makeShared();
}

bool TensorProView::PrintTimeStamps(int offset){
	std::cout << "本包尾列GPS时间戳: ";
	std::cout.fill('0');
	std::cout.width(2);
	std::cout << std::hex << (unsigned int)(unsigned char)m_buf[offset+68];
	std::cout  << " ";
	std::cout.fill('0');
	std::cout.width(2);
	std::cout << std::hex << (unsigned int)(unsigned char)m_buf[offset+69];
	std::cout << " ";
	std::cout.fill('0');
	std::cout.width(2);
	std::cout << std::hex << (unsigned int)(unsigned char)m_buf[offset+70];
	std::cout  << " ";
	std::cout.fill('0');
	std::cout.width(2);
	std::cout << std::hex << (unsigned int)(unsigned char)m_buf[offset+71];    
	std::cout << std::endl;   
	return true; 
}
