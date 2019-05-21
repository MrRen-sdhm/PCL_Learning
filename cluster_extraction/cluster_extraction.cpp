/*
PCL中实现欧式聚类提取。
*/

#include <pcl/ModelCoefficients.h>//模型系数
#include <pcl/point_types.h>//点云基本类型
#include <pcl/io/pcd_io.h>//io
#include <pcl/filters/extract_indices.h>//根据索引提取点云
#include <pcl/filters/voxel_grid.h>//体素格下采样
#include <pcl/features/normal_3d.h>//点云法线特征
#include <pcl/kdtree/kdtree.h>//kd树搜索算法
#include <pcl/sample_consensus/method_types.h>//采样方法
#include <pcl/sample_consensus/model_types.h>//采样模型
#include <pcl/segmentation/sac_segmentation.h>//随机采用分割
#include <pcl/segmentation/extract_clusters.h>//欧式聚类分割
#include <pcl/visualization/pcl_visualizer.h> // 可视化
#include <pcl/filters/statistical_outlier_removal.h>

/******************************************************************************
 打开点云数据，并对点云进行滤波重采样预处理，然后采用平面分割模型对点云进行分割处理
 提取出点云中所有在平面上的点集，并将其存盘
******************************************************************************/
int
main(int argc, char** argv)
{
    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    reader.read("ObjCenterCloud.pcd", *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;


    // 使用欧式聚类的算法、kd树搜索对点云聚类分割
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);                        // 输入点云
    std::vector<pcl::PointIndices> cluster_indices;    // 点云团索引
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec; // 欧式聚类对象
    ec.setClusterTolerance(0.02);                      // 设置近邻搜索的搜索半径为2cm
    ec.setMinClusterSize(100);                         // 设置一个聚类需要的最少的点数目为100
    ec.setMaxClusterSize(25000);                       // 设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod(tree);                          // 设置点云的搜索机制
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);                       //从点云中提取聚类，并将点云索引保存在cluster_indices中

    //迭代访问点云索引cluster_indices，直到分割处所有聚类
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        printf("size:%zu\n", it->indices.size());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]); //获取每一个点云团的点

        cloud_cluster->width =(uint32_t)cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_cluster, false);
        j++;

        // 可视化
        pcl::visualization::PCLVisualizer viewer("3D Viewer");
        viewer.setBackgroundColor(255, 255, 255);
        viewer.initCameraParameters();
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_cluster_handler(cloud_cluster, 0, 255, 0);
        viewer.addPointCloud(cloud_cluster, "cloud_cluster point");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_cluster point");

        while (!viewer.wasStopped()) {
            viewer.spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

    }

    return(0);
}




