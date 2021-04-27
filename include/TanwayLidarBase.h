
/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 30-01-2021
 *  Author: LN

 *  Data processing base class for Tanway all LIDARs   
**************************************************/

#ifndef TANWAYLIDARBASE_H_
#define TANWAYLIDARBASE_H_

#include "Network.h"
#include "LaunchConfig.h"

class TanwayLidarBase
{
public:
	TanwayLidarBase();
	virtual ~TanwayLidarBase();


public:
	bool Initialize(ros::NodeHandle& nh, LaunchConfig& config);
	bool GetUDP();
	static void* GetGPS(void* args);
	void SetBufferAndAnalysis(u_char* data, int len);

public:
	//解析点云数据包
	virtual void AnalysisUDPData()=0;

	//解析GPS数据包
	virtual void AnalysisGPSData()=0;

	//推送点云数据到ROS平台
	virtual void PublishCloud()=0;

public:
	//有效角度范围
	double m_startAngle = 30;
	double m_endAngle = 150;
	//打印GPS协议内容时间戳的标志
 	bool m_gpsProtocolPrintSwitch = true;
	 //打印数据包内的GPS时间戳的标志
 	bool m_timestampPrintSwitch = false;

	//gps data
	u_char m_bufGPS[1500];
	int m_lengthBufGPS = 0;

	//point data
	u_char m_buf[1500];
	int m_lengthBuf = 0;

	//网络管理模块
	UDPNetwork m_UDPNetwork;

protected:
	double RA = 0.01745329;//deg to rad. 180/pi
	double c = 2.997924;
	double m_cos_hA_RA = 0;
	double m_sin_hA_RA = 0;
	double m_tmpCal = 500*c/10.f/16384.f/2;

	ros::Publisher m_rosPublisher;
	std::string m_frameID;

private:
	
	pthread_t thread_tids=-1;
};

#endif