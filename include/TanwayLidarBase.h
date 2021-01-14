
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
	u_char m_buf[1500];
	int m_lengthBuf = 0;
	ros::Publisher m_rosPublisher;

private:
	UDPNetwork m_UDPNetwork;
	
};

#endif