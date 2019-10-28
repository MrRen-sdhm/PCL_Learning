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

int main() {

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    Eigen::Matrix3Xd normals_;

    // load the file
    if (pcl::io::loadPCDFile("/home/sdhm/test.pcd", *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read pcd file\n");
        return (-1);
    }

    while (true) {
        double t_gpu = omp_get_wtime();
        printf("Calculating surface normals ...\n");

        std::cout << "Using integral images for surface normals estimation ...\n";
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
                new pcl::PointCloud<pcl::Normal>);
        pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
        ne.setInputCloud(cloud);
//    ne.setViewPoint(view_points_(0, 0), view_points_(1, 0), view_points_(2, 0));
        ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
        ne.setNormalSmoothingSize(20.0f);
        ne.compute(*cloud_normals);

        // // Visualize them.
        // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
        // viewer->addPointCloud<pcl::PointXYZRGBA>(cloud_processed_, "cloud");
        // // Display one normal out of 20, as a line of length 3cm.
        // viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(cloud_processed_, cloud_normals, 1, 0.03, "normals");
        // while (!viewer->wasStopped())
        // {
        // 	viewer->spinOnce(100);
        // 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        // }

        // Please use release mode to build this C++ project, or you'll get an error:
        // /usr/include/eigen3/Eigen/src/Core/PlainObjectBase.h:258: void Eigen::PlainObjectBase<Derived>::resize(Eigen::Index, Eigen::Index)
        normals_ = cloud_normals->getMatrixXfMap().cast<double>();

        t_gpu = omp_get_wtime() - t_gpu;
        printf("Calculated %zu surface normals in %3.4fs.\n",
               normals_.cols(), t_gpu);
    }

    return 0;
}