#include <iostream>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/visualization/pcl_visualizer.h>

int main() {

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    Eigen::Matrix3Xd normals_;

    // load the file
    if (pcl::io::loadPCDFile("/home/sdhm/tsdf.pcd", *cloud) == -1) {
        PCL_ERROR ("Couldn't read pcd file\n");
        return (-1);
    }
    
    double t_ = omp_get_wtime();
    printf("Calculating surface normals ...\n");

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    if (cloud->isOrganized()) {
        std::cout << "Using integral images for surface normals estimation ...\n";

        pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
        ne.setNormalSmoothingSize(20.0f);
        ne.compute(*cloud_normals);

        normals_ = cloud_normals->getMatrixXfMap().cast<double>();
    } else {
        pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> estimator(6);
//        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_ptr(new pcl::search::KdTree<pcl::PointXYZRGBA>);
        estimator.setInputCloud(cloud);
        estimator.setSearchMethod(tree_ptr);
        estimator.setRadiusSearch(0.03);
        estimator.compute(*cloud_normals);
    }

    t_ = omp_get_wtime() - t_;
    printf("Calculated %zu surface normals in %3.4fs.\n",
           normals_.cols(), t_);


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //设置一个boost共享对象，并分配内存空间
    viewer_ptr->setBackgroundColor(0.0, 0.0, 0.0);//背景黑色
    //viewer.setBackgroundColor (1, 1, 1);//白色
    viewer_ptr->addCoordinateSystem (1.0f, "global");//坐标系
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cloud_color_handler (cloud, 255, 0, 0);//红色
    //viewer->addPointCloud<pcl::PointXYZRGB>(cloud_ptr,cloud_color_handler,"sample cloud");//PointXYZRGB 类型点
    viewer_ptr->addPointCloud<pcl::PointXYZRGBA>(cloud, cloud_color_handler, "original point cloud");//点云标签

    viewer_ptr->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(cloud, cloud_normals, 5, 0.02, "normal");//法线标签
    //参数5表示整个点云中每5各点显示一个法向量（若全部显示，可设置为1，  0.02表示法向量的长度，最后一个参数暂时还不知道 如何影响的）);
    viewer_ptr->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original point cloud");
    //渲染属性，可视化工具，3维数据， 其中PCL_VISUALIZER_POINT_SIZE表示设置点的大小为3

    viewer_ptr->initCameraParameters();//初始化相机参数

    while (!viewer_ptr->wasStopped())
    {
        viewer_ptr->spinOnce ();
        pcl_sleep(0.01);
    }
        

    return 0;
}