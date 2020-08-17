//
// Created by qzj on 2020/8/17.
//

#include "handlePCL.h"

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/cloud_viewer.h>

void projectPCL(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud2) {
//定义模型系数对象coefficients并填充数据
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//参数模型为 ax+by+cz+d=0
// z=0 即为x-y得一个平面
    coefficients->values.resize(4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;
//创建投影滤波对象
    pcl::ProjectInliers <PointType> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud2);
}
