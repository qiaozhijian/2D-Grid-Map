/**
* 函数功能:读取pcl点云文件并发布到topic上去
*/
#include<iostream>
#include<string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include <pcl/common/transforms.h>
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>

using namespace std;
typedef pcl::PointXYZ PointType;

int main(int argc, char **argv) {
    std::string topic, path, frame_id;
    int hz = 5;

    ros::init(argc, argv, "publish_pointcloud");
    ros::NodeHandle nh("~");

    nh.param<std::string>("path", path, "/home/biubiu/Sweeping_robot/resultPointCloudFile.pcd");
    nh.param<std::string>("frame_id", frame_id, "camera");
    nh.param<std::string>("topic", topic, "/pointcloud/output");
    nh.param<int>("hz", hz, 5);

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(topic, 10);

    pcl::PointCloud<PointType> cloud;
    pcl::io::loadPCDFile(path, cloud);

    pcl::PointCloud<PointType>::Ptr StatisticFilteredCloud(new pcl::PointCloud<PointType>());
    pcl::StatisticalOutlierRemoval<PointType> statistical;
    statistical.setInputCloud(static_cast<pcl::PointCloud<PointType>::Ptr>(&cloud));
    statistical.setMeanK(10);                                  //取平均值的临近点数
    statistical.setStddevMulThresh(1);                         //设置判断是否为离群点的阀值
    statistical.filter(*StatisticFilteredCloud);
    pcl::copyPointCloud(*StatisticFilteredCloud,cloud);

    pcl::PointCloud<PointType>::Ptr RomoveGround(new pcl::PointCloud<PointType>());
    pcl::PassThrough<PointType> ptfilter (true); // Initializing with true will allow us to extract the removed indices
    ptfilter.setInputCloud (static_cast<pcl::PointCloud<PointType>::Ptr>(&cloud));
    ptfilter.setFilterFieldName ("y");
    ptfilter.setFilterLimits (-0.01, 1000.0);
    ptfilter.setNegative(true);
    ptfilter.filter (*RomoveGround);
    pcl::copyPointCloud(*RomoveGround,cloud);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f(1, 0, 0)));
    pcl::transformPointCloud(cloud, cloud, transform);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);// 转换成ROS下的数据类型 最终通过topic发布
    output.header.stamp = ros::Time::now();
    output.header.frame_id = frame_id;

    cout << "path = " << path << endl;
    cout << "frame_id = " << frame_id << endl;
    cout << "topic = " << topic << endl;
    cout << "hz = " << hz << endl;

    ros::Rate loop_rate(hz);
    while (ros::ok()) {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}  
