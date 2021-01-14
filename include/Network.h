/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-07-2019
 *  Author: Elodie Shan
 *
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

  socklen_t sockfd, ret, addrlen;
  struct sockaddr_in saddr,caddr;

  u_char buf_[1500];

  std::string host;
  int port;
  std::string LiDARhost;
  int LiDARport;

  // @brief Initialize　some settings.
  bool Init(std::string host_, int port_, std::string LiDARhost_, int LiDARport_);

  // @brief Verify that the connection  is successful.
  bool ConnectValid();

  // @brief Verify that the data source is correct.
  bool SourceValid();

  // @brief Get the UDP data.
  int recvUDP(u_char* buf);

 };


#endif
