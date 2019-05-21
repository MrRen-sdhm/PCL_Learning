#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <math.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

enum
{
    Processor_cl,
    Processor_gl,
    Processor_cpu
};

bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s)
{
    protonect_shutdown = true;
}

// 打印点云数据
void print_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;
}

// VoxelGrid滤波
void VoxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.005f, 0.005f, 0.005f); //设置滤波时创建的体素体积为1cm的立方体
    vg.filter(*cloud);
}

// 统计分析滤波
void statistical_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; //创建滤波器对象
    sor.setInputCloud (cloud);                         //设置待滤波的点云
    sor.setMeanK (50);                                 //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh (1.0);                      //设置判断是否为离群点的阀值
    sor.filter (*cloud);                               //存储
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

int main()
{
    //定义变量
    std::cout << "start!" << std::endl;
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline  *pipeline = 0;

    //搜寻并初始化传感器
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }
    string serial = freenect2.getDefaultDeviceSerialNumber();
    std::cout << "SERIAL: " << serial << std::endl;

    //配置传输格式
#if 1 // sean
    int depthProcessor = Processor_cl;
    if(depthProcessor == Processor_cpu)
    {
        if(!pipeline)
            //! [pipeline]
            pipeline = new libfreenect2::CpuPacketPipeline();
        //! [pipeline]
    }
    else if (depthProcessor == Processor_gl) // if support gl
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        if(!pipeline)
        {
            pipeline = new libfreenect2::OpenGLPacketPipeline();
        }
#else
        std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    }
    else if (depthProcessor == Processor_cl) // if support cl
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if(!pipeline)
            pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
        std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }

    //启动设备
    if(pipeline)
    {
        dev = freenect2.openDevice(serial, pipeline);
    }
    else
    {
        dev = freenect2.openDevice(serial);
    }
    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }
    signal(SIGINT, sigint_handler);
    protonect_shutdown = false;
    libfreenect2::SyncMultiFrameListener listener(
            libfreenect2::Frame::Color |
            libfreenect2::Frame::Depth |
            libfreenect2::Frame::Ir);
    libfreenect2::FrameMap frames;
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    //启动数据传输
    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    //循环接收
    float x, y, z, color;

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

    // 创建黑白点云显示窗口
    pcl::visualization::PCLVisualizer::Ptr viewer_xyz (new pcl::visualization::PCLVisualizer ("xyz"));
    viewer_xyz->setBackgroundColor (0, 0, 0);
    viewer_xyz->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "xyz");
//    viewer_xyz->addCoordinateSystem (1.0);
    viewer_xyz->initCameraParameters ();

    // 创建彩色点云显示窗口
    pcl::visualization::PCLVisualizer::Ptr viewer_rgb (new pcl::visualization::PCLVisualizer ("rgb"));
    viewer_rgb->setBackgroundColor (0, 0, 0);
    viewer_rgb->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rgb");
//    viewer_rgb->addCoordinateSystem (1.0);
    viewer_rgb->initCameraParameters ();

    pcl::visualization::PCLVisualizer viewer("demo");
    int v1(0);
    viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);
//    viewer.setSize(1280, 1024);  // Visualiser window size

    // 创建分割对象
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true); //设置对估计的模型参数进行优化处理
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC); // 设置用哪个随机参数估计方法
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01); // 设置判断是否为模型内点的距离阈值

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    while(!protonect_shutdown)
    {
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

        // 创建PointXYZRGB类型的点云对象
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb ( new pcl::PointCloud<pcl::PointXYZRGB> );

        for (int m = 0;  m < 512 ; m++)
        {
            for (int n = 0 ; n < 424 ; n++)
            {
                pcl::PointXYZRGB p_rgb;

                registration->getPointXYZRGB(&undistorted, &registered, n, m, x, y, z, color);

                const uint8_t *c = reinterpret_cast<uint8_t*>(&color);
                uint8_t b = c[0];
                uint8_t g = c[1];
                uint8_t r = c[2];

                if (z<1.2){ //利用xyz值进行简单滤波
                    p_rgb.z = z;
                    p_rgb.x = x;
                    p_rgb.y = -y;
                    p_rgb.b = b;
                    p_rgb.g = g;
                    p_rgb.r = r;
                }
                cloud_xyzrgb->points.push_back(p_rgb);
            }
        }

        // 创建PointXYZ类型的点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz ( new pcl::PointCloud<pcl::PointXYZ> );
        pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz); // 从彩色点云复制数据

        passthrough_filter(cloud_xyz); // 直通滤波
        // VoxelGrid进行降采样
        VoxelGridFilter(cloud_xyz);
        // 统计分析滤波
        statistical_filter(cloud_xyz);

        // 平面分割
        int i = 0, nr_points = (int)cloud_xyz->points.size();
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Points;
        // While 30% of the original cloud is still there
        while (cloud_xyz->points.size() > 0.3 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
            seg.setInputCloud(cloud_xyz);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0)
            {
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Extract the inliers
            extract.setInputCloud(cloud_xyz);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_p);
            std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
            Points.push_back(cloud_p);

            // 各平面保存为PCD文件
//            std::stringstream ss;
//            ss << "table_scene_lms400_plane_" << i << ".pcd";
//            writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);

            // Create the filtering object
            extract.setNegative(true);
            extract.filter(*cloud_f);
            cloud_xyz.swap(cloud_f); //更新
            i++;
        }

        // The color we will be using
        float bckgr_gray_level = 0.0;  // Black
        float txt_gray_lvl = 1.0 - bckgr_gray_level;
        // Original point cloud is white
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud_xyz, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);

        viewer.removeAllPointClouds();
        viewer.addPointCloud(cloud_xyz, cloud_in_color_h, "cloud_in_v1", v1);

        // 彩色显示分割出的平面
        for (int i = 0; i < Points.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud_p;
            cloud_p = Points.at(i);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out(cloud_p, 255 * (i % 2), 255 * (i % 3), 200);
            char ss[10];
            std::string st = "name";
            snprintf(ss, sizeof(ss), "%d", i);
            st += ss;
            viewer.addPointCloud(cloud_p, cloud_out, st, v1);
        }

        viewer.spinOnce();

        // 更新显示黑白点云
        viewer_xyz->removePointCloud("xyz");
        viewer_xyz->addPointCloud<pcl::PointXYZ> (cloud_xyz, "xyz");
        viewer_xyz->spinOnce();

        // 更新显示彩色点云
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> xyz_rgb(cloud_xyzrgb);
        viewer_rgb->removePointCloud("rgb");
        viewer_rgb->addPointCloud<pcl::PointXYZRGB> (cloud_xyzrgb, xyz_rgb, "rgb");
        viewer_rgb->spinOnce();


        listener.release(frames);

        // 窗口关闭
        if (viewer_rgb->wasStopped() || viewer_xyz->wasStopped()){
            // 保存点云
//            pcl::PCDWriter writer;
//            cloud_xyz->width = 1;
//            cloud_xyz->height = cloud_xyz->points.size();
//            writer.write<pcl::PointXYZ>("Kinect2_XYZ.pcd", *cloud_xyz, false);
//
//            cloud_xyzrgb->width = 1;
//            cloud_xyzrgb->height = cloud_xyz->points.size();
//            writer.write<pcl::PointXYZRGB>("Kinect2_RGB.pcd", *cloud_xyzrgb, false);

            cout << "viewer was closed!" << endl;
            break;
        }

    }

    // 关闭设备
    dev->stop();
    dev->close();

    delete registration;

#endif

    return 0;
}
