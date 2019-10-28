#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// 统计分析滤波
void statistical_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud);
}

void passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud);
    std::cerr << "PointCloud after PassThrough filtering: " << cloud->width * cloud->height
              << " data points." << std::endl;
}

int main(int argc, char** argv)
{
    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    pcl::PCDWriter writer;

#if 0 // 不进行下采样
    // Fill in the cloud data
    reader.read("Kinect2_XYZ.pcd", *cloud_filtered);
#else // 下采样
    // Fill in the cloud data
    reader.read("oil_filter.pcd", *cloud_blob);

    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(0.002f, 0.002f, 0.002f);
    sor.filter(*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);//转化为模板<Template>点云

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
    // Write the downsampled version to disk
    // writer.write<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
#endif
    passthrough_filter(cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true); //设置对估计的模型参数进行优化处理
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC); // 设置用哪个随机参数估计方法
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.001); // 设置判断是否为模型内点的距离阈值 0.01

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int)cloud_filtered->points.size();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > Points;
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
        Points.push_back(cloud_p);

        // 各平面保存为PCD文件
        std::stringstream ss;
        ss << "plane_" << i << ".pcd";
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f); //更新
        i++;


        // 保存局内点索引
        std::vector<int> inliers;
        // 采样一致性模型对象
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_p));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(0.001);
        ransac.computeModel();
        ransac.getInliers(inliers);

        std::cout << "局内点：" << inliers.size() << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
        final->resize(inliers.size());

        pcl::copyPointCloud(*cloud_p, inliers, *final);
        pcl::io::savePCDFile("2.pcd", *final);

        Eigen::VectorXf coef = Eigen::VectorXf::Zero(4 , 1);
        ransac.getModelCoefficients(coef);

        std::cout << coef << std::endl;

    }
    pcl::visualization::PCLVisualizer viewer("demo");
    int v1(0);
    viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud_filtered, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
    viewer.addPointCloud(cloud_filtered, cloud_in_color_h, "cloud_in_v1", v1);

    // 彩色显示分割出的平面
    for (int i = 0; i < Points.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud_p;
        cloud_p = Points.at(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out(cloud_p, 255 * (i % 2), 255 * (i % 3), 155);
        char ss[10];
        std::string st = "name";
        snprintf(ss, sizeof(ss), "%d", i);
        st += ss;
        viewer.addPointCloud(cloud_p, cloud_out, st, v1);
    }
    viewer.setSize(1280, 1024);  // Visualiser window size

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return (0);
}