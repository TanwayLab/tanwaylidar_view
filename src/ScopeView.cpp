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
#include "ScopeView.h"


ScopeView::ScopeView()
{
	TanwayViewPointCloud cloud;
	m_tanwayViewPointCloud = cloud.makeShared();
}
ScopeView::~ScopeView(){};

void ScopeView::AnalysisUDPData()
{
	int blocks_num = 0;

	while (blocks_num < 8) 
	{
		int offset = blocks_num * 140;

		if (m_timestampPrintSwitch)
			PrintTimeStamps(offset);

		//水平角度 （索引：128-131）
		int hextoAngle = FourHexToInt(m_buf[offset+128],m_buf[offset+129],m_buf[offset+130],m_buf[offset+131]);
		float horAngle = hextoAngle/100000.0f;
		
		//索引拍值
		unsigned char  chaIndex = m_buf[offset + 136];
		unsigned short HextoSepIndex = chaIndex >> 6 | 0X0000;
		//ABC镜面值
		unsigned char  faceIndex = m_buf[offset + 137];
		unsigned short FaceSepIndex = (faceIndex << 2) >> 6 | 0X0000;

		//按照角度范围
		if(horAngle>=m_startAngle && horAngle<=m_endAngle)
		{
			//重置分帧标志
			m_needPublishCloud = true;

			int seq = 0;
			while (seq < 16) 
			{
				//单回波 距离+脉宽
				double hexToInt = TwoHextoInt(m_buf[offset + seq * 8 + 0], m_buf[offset + seq * 8 + 1]);
				float pulse = TwoHextoInt(m_buf[offset + seq * 8 + 2], m_buf[offset + seq * 8 + 3]);
				
				//双回波 距离+脉宽（暂时没用到）
				double hexToInt2 = TwoHextoInt(m_buf[offset + seq * 8 + 4], m_buf[offset + seq * 8 + 5]);
				float pulse2 = TwoHextoInt(m_buf[offset + seq * 8 + 6], m_buf[offset + seq * 8 + 7]);

				//距离
				float L = hexToInt*500*c/10.f/16384.f/2;
				
				//距离过滤
				if (L<=0 || L > 200) 
				{
					seq++;
					continue;
				}

				//通道	
				int channel = 16 * (blocks_num >= 4? blocks_num-4:blocks_num) + seq + 1;
				//脉宽值
				float pulsewidth = pulse*500*c/10.f/16384.f/2;
				
				//计算点数据
				UseAnalysisPoint(horAngle, L, channel, pulsewidth, HextoSepIndex, FaceSepIndex);

				seq++;
			}

		}
		//else
		//	ROS_DEBUG("hA = [%f]",horizontalAngle);

		if (horAngle < m_startAngle && m_needPublishCloud)
		{
			PublishCloud();
			m_needPublishCloud = false;
		}

		blocks_num++;
	}
}

bool ScopeView::UseAnalysisPoint(float horAngle, float L, int channel, float pulseWidth, int sepIndex, int faceIndex)
{
 	float cos_hA_RA = cos(horAngle * RA);
	float sin_hA_RA = sin(horAngle * RA);
	float vA = verticalChannels[channel-1];
	float cos_vA_RA = cos(vA * RA);

	TanwayViewPoint point;
	point.x = L * cos_vA_RA * cos_hA_RA;
	point.y = L * cos_vA_RA * sin_hA_RA;
	point.z = L * sin(vA * RA);
	point.pulsewidth = pulseWidth;

	m_tanwayViewPointCloud->points.push_back(point);
}

void ScopeView::PublishCloud()
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

bool ScopeView::PrintTimeStamps(int offset){
	std::cout << "本包尾列GPS时间戳: ";
	std::cout.fill('0');
	std::cout.width(2);
	std::cout << std::hex << (unsigned int)(unsigned char)m_buf[offset+132];
	std::cout  << " ";
	std::cout.fill('0');
	std::cout.width(2);
	std::cout << std::hex << (unsigned int)(unsigned char)m_buf[offset+133];
	std::cout << " ";
	std::cout.fill('0');
	std::cout.width(2);
	std::cout << std::hex << (unsigned int)(unsigned char)m_buf[offset+134];
	std::cout  << " ";
	std::cout.fill('0');
	std::cout.width(2);
	std::cout << std::hex << (unsigned int)(unsigned char)m_buf[offset+135];    
	std::cout << std::endl;   
	return true; 
}
