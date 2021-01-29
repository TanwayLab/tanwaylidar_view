
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
	void SetBufferAndAnalysis(u_char* data, int len);

protected:
	//解析UDP数据包
	virtual void AnalysisUDPData()=0;

	//推送点云数据到ROS平台
	virtual void PublishCloud()=0;

public:
	//有效角度范围
	double m_startAngle = 30;
	double m_endAngle = 150;
	//打印GPS时间戳的标志
 	bool m_timestampPrintSwitch = false;

protected:
	double RA = 0.01745329;//deg to rad. 180/pi
	double c = 2.997924;
	double m_cos_hA_RA = 0;
	double m_sin_hA_RA = 0;
	double m_tmpCal = 500*c/10.f/16384.f/2;

	u_char m_buf[1500];
	int m_lengthBuf = 0;
	ros::Publisher m_rosPublisher;

private:
	UDPNetwork m_UDPNetwork;
	
};

#endif