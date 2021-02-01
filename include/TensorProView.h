/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 30-01-2021
 *  Author: LN

 *  Data processing for Tanway TensorPro LIDARs   
**************************************************/


#ifndef TENSORPROVIEW_H_
#define TENSORPROVIEW_H_

#include "TanwayLidarBase.h"
#include <sensor_msgs/PointCloud2.h> //ROS message type to publish a pointCloud
#include "TanwayViewPointType.h"


class TensorProView : public TanwayLidarBase
{
public:
	TensorProView();

	virtual ~TensorProView();

protected:
	//应用原始点的UDP信息计算点信息
	virtual void UseAnalysisPoint(float horAngle, float hexL, int channel, float hexPulseWidth, int offset);
	//推送点云数据到ROS平台
	virtual void PublishCloud();
	

protected:
 	double verticalChannels[16] = 
	 { -5.274283f, -4.574258f, -3.872861f, -3.1703f, -2.466783f, -1.762521f, -1.057726f, -0.352611f, 0.352611f,
 		1.057726f, 1.762521f, 2.466783f, 3.1703f, 3.872861f, 4.574258f, 5.274283f};


	//Ros点云结构
	sensor_msgs::PointCloud2 m_rosPointCloud; 

private:
	//解析UDP数据包
	void AnalysisUDPData();
	//打印GPS时间戳
	bool PrintTimeStamps(int offset);

private:
	//PCL点云结构体
	TanwayViewPointCloud::Ptr m_tanwayViewPointCloud;
	
	//分点云帧的标志
	bool m_needPublishCloud = true;


};

#endif
