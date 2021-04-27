/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 30-01-2021
 *  Author: LN

 *  Data processing for Tanway TensorPro LIDARs   
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

//解析GPS数据包
void TensorProView::AnalysisGPSData()
{
	//确定GPS包长度正确
	if (m_lengthBufGPS == 120 && m_gpsProtocolPrintSwitch)
		UseAnalysisGPS(m_bufGPS);
}

//解析点云数据包
void TensorProView::AnalysisUDPData()
{
	//确定数据包长度正确
	if (m_lengthBuf == 120 && m_gpsProtocolPrintSwitch)
	{
		//避免修改雷达数据发送端口和GPS端口一样，导致的GPS接收问题
		UseAnalysisGPS(m_buf);
		return;
	}

	//不符合数据包长度，直接丢弃
	if (m_lengthBuf != 1440) return;

	//解析数据包
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
	float vA = verticalChannels[channel-1];
	float cos_vA_RA = cos(vA * RA);
	float L = hexL*m_tmpCal;
	float pulseWidth = hexPulseWidth*m_tmpCal;

	//按有效距离过滤
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
	m_tanwayViewPointCloud->header.frame_id = m_frameID; //Point cloud coordinate system name
	
	pcl::toROSMsg(*m_tanwayViewPointCloud, m_rosPointCloud); //convert between PCL and ROS datatypes
	m_rosPointCloud.header.stamp = ros::Time::now(); //Get ROS system time
	m_rosPublisher.publish(m_rosPointCloud); //Publish cloud
	
	TanwayViewPointCloud cloud;
	m_tanwayViewPointCloud = cloud.makeShared();
}

void TensorProView::UseAnalysisGPS(u_char* buffer)
{
	//定位状态：有效定位0x41  无效定位0x56
	std::string gps_status = "STATUS: (Unknown)";
	bool bUunknown = false;
	if (buffer[3] == 0x41)
		gps_status = "STATUS: (Valid)";
	else if (buffer[3] == 0x56)
		gps_status = "STATUS: (Invalid)";
	else
	{
		gps_status = "STATUS: (Unknown)";
		bUunknown = true;
	}

	//GPS时间信息
	//                   0    1    2    3    4    5    6   7    8    9    10   11   12   13   14   15   16   17   18    19
	char sz_gps[20] = { '2', '0', ' ', ' ', '-', ' ', ' ', '-', ' ', ' ', ' ', ' ', ' ', ':', ' ', ' ', ':', ' ', ' ', '\0' };

	sz_gps[2] = bUunknown ? '#' : buffer[4]; //year
	sz_gps[3] = bUunknown ? '#' : buffer[5];
	sz_gps[5] = bUunknown ? '#' : buffer[6]; //month
	sz_gps[6] = bUunknown ? '#' : buffer[7];
	sz_gps[8] = bUunknown ? '#' : buffer[8]; //day
	sz_gps[9] = bUunknown ? '#' : buffer[9];

	sz_gps[11] = bUunknown ? '#' : buffer[10]; //hour
	sz_gps[12] = bUunknown ? '#' : buffer[11];
	sz_gps[14] = bUunknown ? '#' : buffer[12]; //minute
	sz_gps[15] = bUunknown ? '#' : buffer[13];
	sz_gps[17] = bUunknown ? '#' : buffer[14]; //second
	sz_gps[18] = bUunknown ? '#' : buffer[15];


	//GPS协议帧时间
	unsigned int time_us = FourHexToInt(buffer[16], buffer[17], buffer[18], buffer[19]);
	
	std::string gps_value = gps_status + " GPS: " + sz_gps;
	std::cout << gps_value << std::endl;
}

bool TensorProView::PrintTimeStamps(int offset)
{
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
