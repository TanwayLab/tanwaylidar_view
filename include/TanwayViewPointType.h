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