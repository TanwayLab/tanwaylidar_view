/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 30-01-2021
 *  Author: LN

 *  Point cloud data structure definition for view project   
**************************************************/

#ifndef TANWAYVIEWPOINTTYPE_H_
#define TANWAYVIEWPOINTTYPE_H_

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct TanwayViewPoint
{
  PCL_ADD_POINT4D;
  float pulsewidth;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


POINT_CLOUD_REGISTER_POINT_STRUCT(TanwayViewPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, pulsewidth, pulsewidth)
                                 )

typedef pcl::PointCloud<TanwayViewPoint> TanwayViewPointCloud;

#endif