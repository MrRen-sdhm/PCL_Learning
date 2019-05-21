//
// Created by sdhm on 19-2-25.
//

#ifndef PCL_KINECT2_MY_PCL_TOOLS_H
#define PCL_KINECT2_MY_PCL_TOOLS_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/conversions.h>

// 打印点云数据
void print_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " "    << cloud->points[i].y
                  << " "    << cloud->points[i].z << std::endl;
}

// 直通滤波
void passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::cerr << "PointCloud before PassThrough filtering: " << cloud->width * cloud->height
              << " data points." << std::endl;
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.2, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud);
    std::cerr << "PointCloud after PassThrough filtering: " << cloud->width * cloud->height
              << " data points." << std::endl;
}

// VoxelGrid滤波
//void voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//    // Create the VoxelGrid filtering object
//    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
//    pcl::toPCLPointCloud2(*cloud, *cloud_filtered);
//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//    sor.setInputCloud (cloud_filtered);
//    sor.setLeafSize (0.005f, 0.005f, 0.005f);
//    sor.filter (*cloud_filtered);
//    pcl::fromPCLPointCloud2(*cloud_filtered, *cloud);
//}

// 体素格滤波下采样
void voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.005f, 0.005f, 0.005f); //设置滤波时创建的体素体积为1cm的立方体
    vg.filter(*cloud);
}

// 统计分析滤波
void statistical_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // 创建滤波器，对每个点分析的临近点的个数设置为50，并将标准差的倍数设置为1，这意味着如果一
    // 个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; //创建滤波器对象
    sor.setInputCloud (cloud);                         //设置待滤波的点云
    sor.setMeanK (50);                                 //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh (1.0);                      //设置判断是否为离群点的阀值
    sor.filter (*cloud);                               //存储
}

#endif //PCL_KINECT2_MY_PCL_TOOLS_H
