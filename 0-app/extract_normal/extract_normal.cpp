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
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr surface_cloud_with_normal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // load the file
    if (pcl::io::loadPCDFile("../surface_cloud.pcd", *surface_cloud_with_normal) == -1 ||
        pcl::io::loadPCDFile("../cloud_with_normal.pcd", *cloud_with_normal) == -1) {
        PCL_ERROR ("Couldn't read pcd file\n");
        return (-1);
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*cloud_with_normal,*normals);
    pcl::copyPointCloud(*cloud_with_normal,*mesh_cloud);

    // kdtree对象
    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
    // 输入点云
    kdtree.setInputCloud (cloud_with_normal);
    // 随机定义一个 需要搜寻的点  创建一个searchPoint变量作为查询点
//    pcl::PointXYZRGBNormal searchPoint;
//    searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
//    searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
//    searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

    // K 个最近点去搜索 nearest neighbor search
    int K = 1;
    // 两个向量来存储搜索到的K近邻，两个向量中，一个存储搜索到查询点近邻的索引，另一个存储对应近邻的距离平方
    std::vector<int> pointIdxNKNSearch(K);//最近临搜索得到的索引
    std::vector<float> pointNKNSquaredDistance(K);//平方距离

    for(int i = 0; i < surface_cloud_with_normal->size(); ++i)
//    for(int i = 0; i < 3; ++i)
    {
        pcl::PointXYZRGBNormal *itSP = &surface_cloud_with_normal->points[i];

        std::cout << "K nearest neighbor search at (" << itSP->x
              << " " << itSP->y
              << " " << itSP->z
              << ") with K=" << K << std::endl;

        if ( kdtree.nearestKSearch (*itSP, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j)
                std::cout << " " << cloud_with_normal->points[ pointIdxNKNSearch[j] ].x
                          << " " << cloud_with_normal->points[ pointIdxNKNSearch[j] ].y
                          << " " << cloud_with_normal->points[ pointIdxNKNSearch[j] ].z
                          << " (squared distance: "
                          << pointNKNSquaredDistance[j]
                          << ")"
                          << std::endl;
            itSP->normal_x = cloud_with_normal->points[ pointIdxNKNSearch[0] ].normal_x;
            itSP->normal_y = cloud_with_normal->points[ pointIdxNKNSearch[0] ].normal_y;
            itSP->normal_z = cloud_with_normal->points[ pointIdxNKNSearch[0] ].normal_z;
        }
    }

    pcl::PointCloud<pcl::Normal>::Ptr surface_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr surface_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*surface_cloud_with_normal,*surface_normals);
    pcl::copyPointCloud(*surface_cloud_with_normal,*surface_cloud);

//    std::cout << "K nearest neighbor search at (" << searchPoint.x
//              << " " << searchPoint.y
//              << " " << searchPoint.z
//              << ") with K=" << K << std::endl;
//
//    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//    {
//        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
//            std::cout << " " << cloud_with_normal->points[ pointIdxNKNSearch[i] ].x
//                      << " " << cloud_with_normal->points[ pointIdxNKNSearch[i] ].y
//                      << " " << cloud_with_normal->points[ pointIdxNKNSearch[i] ].z
//                      << " (squared distance: "
//                      << pointNKNSquaredDistance[i]
//                      << ")"
//                      << std::endl;
//    }

//    for(int r = 0; r < depth.rows; ++r)
//    {
//        pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
//        const uint16_t *itD = depth.ptr<uint16_t>(r);
//        const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
//        const float y = lookupY.at<float>(0, r);
//        const float *itX = lookupX.ptr<float>();
//
//        for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
//        {
//            register const float depthValue = *itD / 1000.0f;
//            // Check for invalid measurements
//            if(*itD == 0 || *itD > 1000)
//            {
//                // not valid
//                itP->x = itP->y = itP->z = badPoint;
//                itP->rgba = 0;
//                continue;
//            }
//            itP->z = depthValue;
//            itP->x = *itX * depthValue;
//            itP->y = y * depthValue;
//            itP->b = itC->val[0];
//            itP->g = itC->val[1];
//            itP->r = itC->val[2];
//            itP->a = 255;
//        }
//    }


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //设置一个boost共享对象，并分配内存空间
    viewer_ptr->setBackgroundColor(0, 0, 0);//背景黑色
//    viewer_ptr->setBackgroundColor (1, 1, 1);//白色
    viewer_ptr->addCoordinateSystem (0.1f, "global");//坐标系
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_color_handler (surface_cloud_with_normal, 255, 0, 0);//红色
    viewer_ptr->addPointCloud<pcl::PointXYZRGBNormal>(surface_cloud_with_normal, cloud_color_handler, "original point cloud");

    viewer_ptr->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(surface_cloud, surface_normals, 1, 0.02, "normal");
    //参数5表示整个点云中每5各点显示一个法向量（若全部显示，可设置为1，0.02表示法向量的长度）;
    viewer_ptr->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original point cloud");
    //渲染属性，可视化工具，3维数据， 其中PCL_VISUALIZER_POINT_SIZE表示设置点的大小为3

    viewer_ptr->initCameraParameters();//初始化相机参数

    ///
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr1 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //设置一个boost共享对象，并分配内存空间
    viewer_ptr1->setBackgroundColor(0, 0, 0);//背景黑色
//    viewer_ptr->setBackgroundColor (1, 1, 1);//白色
    viewer_ptr1->addCoordinateSystem (0.1f, "global");//坐标系
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_color_handler1 (cloud_with_normal, 255, 0, 0);//红色
    viewer_ptr1->addPointCloud<pcl::PointXYZRGBNormal>(cloud_with_normal, cloud_color_handler1, "point cloud");

    viewer_ptr1->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(mesh_cloud, normals, 1, 0.02, "normals");
    //参数5表示整个点云中每5各点显示一个法向量（若全部显示，可设置为1，0.02表示法向量的长度）;
    viewer_ptr1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point cloud");
    //渲染属性，可视化工具，3维数据， 其中PCL_VISUALIZER_POINT_SIZE表示设置点的大小为3

    viewer_ptr1->initCameraParameters();//初始化相机参数

    while (!viewer_ptr->wasStopped())
    {
        viewer_ptr->spinOnce ();
        viewer_ptr1->spinOnce ();
        pcl_sleep(0.01);
    }
        

    return 0;
}