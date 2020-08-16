/*************************************************************************
	> File Name: src/pcd2octomap.cpp
	> Author: Gao Xiang
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年12月12日 星期六 15时51分45秒
 ************************************************************************/

#include <iostream>
#include <assert.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

//octomap 
#include <octomap/octomap.h>
using namespace std;

int main( int argc, char** argv )
{
    if (argc != 4)
    {
        cout<<"Usage: pcd2octomap <input_file> <output_file>"<<endl;
        return -1;
    }
    double resolution = 0.05;
    resolution = stod(argv[1]);
    resolution = 0.1;

    string input_file = argv[2], output_file = argv[3];
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    pcl::io::loadPCDFile<pcl::PointXYZRGBA> ( input_file, *cloud );
    cout<<"point cloud loaded, piont size = "<<cloud->points.size()<<endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr StatisticFilteredCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> statistical;
    statistical.setInputCloud(cloud);
    statistical.setMeanK(100);                                  //取平均值的临近点数
    statistical.setStddevMulThresh(1);                         //设置判断是否为离群点的阀值
    statistical.filter(*StatisticFilteredCloud);


    //声明octomap变量
    cout<<"copy data into octomap..."<<endl;
    // 创建八叉树对象，参数为分辨率，这里设成了0.05
    cout<<"tree resolution: "<<resolution<<endl;
    octomap::OcTree tree( resolution );

    for (auto p:cloud->points)
    {
        // 将点云里的点插入到octomap中
        //cout<<p.x<<" "<<p.y<<" "<<p.z<<endl;
        if(p.y < 0.05 )
            tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }

    // 更新octomap
    tree.updateInnerOccupancy();
    // 存储octomap
    tree.writeBinary( output_file );
    cout<<"done."<<endl;

    return 0;
}
