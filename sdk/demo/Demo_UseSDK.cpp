/*
* Software License Agreement (BSD License)
*   
*  Copyright (c) Tanway science and technology co., LTD.
*
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without modification, 
*  are permitted provided  that the following conditions are met:
*
*   1.Redistributions of source code must retain the above copyright notice, 
*     this list of conditions and the following disclaimer.
*
*   2.Redistributions in binary form must reproduce the above copyright notice, 
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*     
*   3.Neither the name of the copyright holder(s) nor the names of its  contributors
*     may be used to endorse or promote products derived from this software without 
*     specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
*  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
*  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "../TanwayLidarSDK.h"

//point struct
struct PointXYZ
{
	float x;
	float y;
	float z;
	int channel;
	//float intensity;
	unsigned int t_sec;         /* The value represents seconds since 1900-01-01 00:00:00 (the UNIX epoch).*/ 
	unsigned int t_usec;        /* remaining microseconds */
};

void pointCloudCallback(TWPointCloud<PointXYZ>::Ptr pointCloud)
{
	/*
	*The point cloud struct uses a smart pointer. 
	*Please copy the point cloud data to another thread for use.
	*Avoid directly operating the UI in the callback function.
	*/
	std::cout << "width:" << pointCloud->width 
			  << " height:" << pointCloud->height 
			  << " point cloud size: " << pointCloud->Size() << std::endl;
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

int main()
{
	/*
	*Real-time connection using lidar. 
	*If you have PCL installed, you can also use the point type 'pcl::PointXYZI' like this 'TanwayLidarSDK<pcl::PointXYZI> lidar';
	*/
	TanwayLidarSDK<PointXYZ> lidar("192.168.111.51", "192.168.111.204", 5600, LT_TensorPro);
	lidar.RegPointCloudCallback(pointCloudCallback);
	lidar.RegGPSCallback(gpsCallback);
	lidar.RegExceptionCallback(exceptionCallback);
	lidar.Start();

	/*
	*using pcap file to replay.
	*/
	//TanwayLidarSDK<PointXYZ> lidar("./5678_32.pcap", "192.168.111.51", 5600, LT_TSP0332, true);
	//lidar.RegPointCloudCallback(pointCloudCallback);
	//lidar.RegGPSCallback(gpsCallback);
	//lidar.RegExceptionCallback(exceptionCallback);
	//lidar.SetCorrectedAngleToScope192(0.0f, 0.1f, 0.2f);
	//lidar.SetCorrectedAngleToTSP0332(0.0f, -6.0f);
	//lidar.Start();
	

	//quit
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

    return 0;
}

