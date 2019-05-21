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

#include <opencv2/opencv.hpp>

#include "my_pcl_tools.h"

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

    //循环接收
    float x, y, z, color;
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

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

//                if (z>0.2 && z<1.2 && y>-0.2 && x>-0.2){ //利用xyz值进行简单滤波
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
        voxel_grid_filter(cloud_xyz); // VoxelGrid进行降采样
        statistical_filter(cloud_xyz); // 统计分析滤波

        // 更新显示黑白点云
        viewer_xyz->removeAllPointClouds();
        viewer_xyz->addPointCloud<pcl::PointXYZ> (cloud_xyz, "xyz");
        viewer_xyz->spinOnce (100);

        // 更新显示彩色点云
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> xyz_rgb(cloud_xyzrgb);
        viewer_rgb->removeAllPointClouds();
        viewer_rgb->addPointCloud<pcl::PointXYZRGB> (cloud_xyzrgb, xyz_rgb, "rgb");
        viewer_rgb->spinOnce (100);


        listener.release(frames);

        // 窗口关闭
        if (viewer_rgb->wasStopped() || viewer_xyz->wasStopped()){
            // 保存点云
            pcl::PCDWriter writer;
            cloud_xyz->width = 1;
            cloud_xyz->height = cloud_xyz->points.size();
            writer.write<pcl::PointXYZ>("Kinect2_XYZ.pcd", *cloud_xyz, false);
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
