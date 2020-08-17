//
// Created by qzj on 2020/8/17.
//

#ifndef OCTOMAP_TUTOR_MODELCOEFFICIENTS_H
#define OCTOMAP_TUTOR_MODELCOEFFICIENTS_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointType;

void projectPCL(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud2);

#endif //OCTOMAP_TUTOR_MODELCOEFFICIENTS_H
