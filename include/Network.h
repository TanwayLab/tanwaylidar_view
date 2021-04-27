/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 30-01-2021
 *  Author: LN

 *  UDP interface for Tanway Tensor 3D LIDARs
**************************************************/

#ifndef NETWORK_H_
#define NETWORK_H_

#include <ros/ros.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>  
#include <strings.h>


class UDPNetwork
{
public:
	UDPNetwork();
	virtual ~UDPNetwork();


	// @brief Initialize　some settings.
	bool Init(std::string host_, int port_, std::string LiDARhost_, int LiDARport_);

	// @brief Get the Point data.
	int recvPoint(u_char* buf);

	// @brief Get the GPS data.
	int recvGPS(u_char* buf);

private:
	// @brief Verify that the connection  is successful.
	bool ConnectValid();

	// @brief Verify that the data source is correct.
	bool SourceValid();

	socklen_t m_socketPoint;
	socklen_t m_socketGPS;

	std::string host;
	int port;
	std::string LiDARhost;
	int LiDARport;

 };


#endif
