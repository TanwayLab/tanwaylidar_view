/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 30-01-2021
 *  Author: LN

 *  UDP interface for Tanway Tensor 3D LIDARs
**************************************************/

#include "Network.h"
#include <iostream> 
#include <stdlib.h>
#include <stdio.h>
using namespace std;
UDPNetwork::UDPNetwork(){};

UDPNetwork::~UDPNetwork(){};

bool UDPNetwork::Init(std::string host_, int port_, std::string LiDARhost_, int LiDARport_)
{
	host = host_;
	port = port_;
	LiDARhost = LiDARhost_;
	LiDARport = LiDARport_;

	m_socketPoint = socket(AF_INET,SOCK_DGRAM,0);

	struct sockaddr_in saddr_point;
	saddr_point.sin_family = AF_INET; //IPv4
	saddr_point.sin_port = htons(port);
	saddr_point.sin_addr.s_addr = inet_addr(host.data());//Convert between the binary IP address of the 32-bit network byte and the dotted decimal IP address

	int ret = bind(m_socketPoint,(struct sockaddr*)&saddr_point,sizeof(saddr_point)); //Assign a local name to an unnamed socket to establish a local bundle (host address/port number) for the socket.
	if(ret < 0) 
	{

		perror("Bind data port fail!");
		return false;
	}

	bool status = ConnectValid() && SourceValid();
	if (status)
	{
		m_socketGPS = socket(AF_INET,SOCK_DGRAM,0);

		struct sockaddr_in saddr_gps;
		saddr_gps.sin_family = AF_INET; //IPv4
		saddr_gps.sin_port = htons(10110); //固定GPS接收端口
		saddr_gps.sin_addr.s_addr = inet_addr(host.data());//Convert between the binary IP address of the 32-bit network byte and the dotted decimal IP address

		int ret = bind(m_socketGPS, (struct sockaddr*)&saddr_gps, sizeof(saddr_gps)); //Assign a local name to an unnamed socket to establish a local bundle (host address/port number) for the socket.
		
		if(ret < 0) 
		{
			perror("Bind gps port:10110 fail!");
		}
	}
		
	return status;

}

bool UDPNetwork::ConnectValid()
{
	int RECV_TIMEOUT_COUNT = 0;
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(m_socketPoint,&readfds);
	struct timeval RECV_TIMEOUT;
	RECV_TIMEOUT.tv_sec = 10;
	while(select(m_socketPoint+1,&readfds,NULL,NULL,&RECV_TIMEOUT)==0)//Listening to the state of the socket
	{
		if (RECV_TIMEOUT_COUNT == 5)
			return false;
		ROS_WARN("Failed to connect with LiDAR , time out!");
		sleep(1);
		RECV_TIMEOUT_COUNT++;
		FD_ZERO(&readfds);
		FD_SET(m_socketPoint,&readfds);
		RECV_TIMEOUT.tv_sec = 30;
	} 

	return true;
}

bool UDPNetwork::SourceValid()
{
  // Check LiDAR data source
	u_char buf_[1500];
	bzero(buf_,sizeof(buf_));

	struct sockaddr_in saddr_point;
	socklen_t addrlen = sizeof(saddr_point);

	int ret = recvfrom(m_socketPoint, buf_, 1500, 0, (struct sockaddr*)&saddr_point, &addrlen); //Recieve UDP packet to update client address.

	if (inet_ntoa(saddr_point.sin_addr)!= LiDARhost/* || htons(caddr.sin_port)!= LiDARport*/)
	{
		ROS_ERROR("Warn data source %s: %d.  The valid host is %s: %d ",(char *)inet_ntoa(saddr_point.sin_addr),htons(saddr_point.sin_port),LiDARhost.data(),LiDARport);
		ROS_ERROR("Please check the LiDAR or the config file!");
		return false; 
	}
	return true;
}

int UDPNetwork::recvPoint(u_char* buf)
{
	struct sockaddr_in point_addr;
	bzero(buf,sizeof(buf));
	socklen_t addrlen = sizeof(point_addr);

	int ret = recvfrom(m_socketPoint, buf, 1500, 0, (struct sockaddr*)&point_addr, &addrlen); //Recieve UDP packets

	return ret;
}

int UDPNetwork::recvGPS(u_char* buf)
{
	struct sockaddr_in gps_addr;
	bzero(buf,sizeof(buf));
	socklen_t gps_addrlen = sizeof(gps_addr);

	int ret = recvfrom(m_socketGPS, buf, 1500, 0, (struct sockaddr*)&gps_addr, &gps_addrlen); //Recieve UDP packets

	return ret;
}


