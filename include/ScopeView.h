/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 30-01-2021
 *  Author: LN

 *  Data processing for Tanway Scope LIDARs   
**************************************************/

#ifndef SCOPEVIEW_H_
#define SCOPEVIEW_H_

#include "TanwayLidarBase.h"
#include <sensor_msgs/PointCloud2.h> //ROS message type to publish a pointCloud
#include "TanwayViewPointType.h"


class ScopeView : public TanwayLidarBase
{
public:
	ScopeView();

	virtual ~ScopeView();

protected:
	//应用原始点的UDP信息计算点信息
	virtual void UseAnalysisPoint(int echo, int sepIndex, int faceIndex, float horAngle, int channel, float hexL, float hexPulseWidth);
		//解析并打印打印GPS时间戳
	virtual void UseAnalysisGPS(u_char* buffer);
	//推送点云数据到ROS平台
	virtual void PublishCloud();
	

protected:
	double verticalChannels[64] = {
		-14.64, -14.17, -13.69, -13.22, -12.75, -12.28, -11.81, -11.34, -10.87, -10.40, -9.93, -9.47, -9.00, -8.54, -8.07, -7.61, -7.14, -6.68, -6.22, -5.76, -5.29, -4.83, -4.37, -3.91, -3.45, -2.99, -2.53, -2.07, -1.61, -1.15, -0.69, -0.23,
		0.23, 0.69, 1.15, 1.61, 2.07, 2.53, 2.99, 3.45, 3.91, 4.37, 4.83, 5.29, 5.76, 6.22, 6.68, 7.14, 7.61, 8.07, 8.54, 9.00, 9.47, 9.93, 10.40, 10.87, 11.34, 11.81, 12.28, 12.75, 13.22, 13.69, 14.17, 14.64};

	//重复记录第一拍的水平角度
	float m_firstSeparateAngle = -1.0;

	//Ros点云结构
	sensor_msgs::PointCloud2 m_rosPointCloud; 

private:
	//解析GPS数据包
	void AnalysisGPSData();
	//解析UDP数据包
	void AnalysisUDPData();
	//打印数据包中的GPS信息
	bool PrintTimeStamps(int offset);

private:
	//PCL点云结构体
	TanwayViewPointCloud::Ptr m_tanwayViewPointCloud;
	
	//分点云帧的标志
	bool m_needPublishCloud = true;

};

#endif
