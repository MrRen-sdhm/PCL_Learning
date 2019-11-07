#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

void readCameraInfo(cv::Mat &cameraMatrix)
{
    // cobot
//    std::vector<double> cameraMatrixVec = {1.0751836750739103e+03, 0., 9.9204536064492709e+02,
//                                           0., 1.0787798824980591e+03, 5.5685612287788467e+02,
//                                           0., 0., 1.};
    // kinect
//    std::vector<double> cameraMatrixVec = {1.0698993215003607e+03, 0., 9.4663298692405795e+02, 0.,
//                                           1.0700923327220949e+03, 5.4185806716826630e+02, 0., 0., 1.};
    // realsense
//    std::vector<double> cameraMatrixVec = {6.105422363281250000e+02, 0.0, 3.223493041992187500e+02,
//                                           0.0, 6.107111206054687500e+02, 2.464488830566406250e+02,
//                                           0.0, 0.0, 1.0};
    // xtion
//    std::vector<double> cameraMatrixVec = {533.6422696034836, 0.0, 319.4091030774892,
//                                           0.0, 534.7824445233571, 236.4374299691866,
//                                           0.0, 0.0, 1.0};

    std::vector<double> cameraMatrixVec = {5.542546911911870211e+02, 0.0, 3.205000000000000000e+02,
                                           0.0, 5.542546911911870211e+02, 2.405000000000000000e+02,
                                           0.0, 0.0, 1.0};

    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
        *itC = cameraMatrixVec[i];
    }
}

void createLookup(cv::Mat &lookupX, cv::Mat &lookupY, size_t width, size_t height)
{
    cv::Mat cameraMatrixColor;
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);

    readCameraInfo(cameraMatrixColor);

    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
        *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
        *it = (c - cx) * fx;
    }
}

// 创建点云 color(1920*1080) depth(1920*1080) cloud(1920*1080)
void create_point_cloud(const cv::Mat &color ,const cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                                cv::Mat lookupX, cv::Mat lookupY) {
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

#pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
        pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
        const uint16_t *itD = depth.ptr<uint16_t>(r);
        const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
        const float y = lookupY.at<float>(0, r);
        const float *itX = lookupX.ptr<float>();

        for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
        {
            register const float depthValue = *itD / 1000.0f;
            // Check for invalid measurements
            if(*itD == 0 || *itD > 1000)
            {
                // not valid
                itP->x = itP->y = itP->z = badPoint;
                itP->rgba = 0;
                continue;
            }
            itP->z = depthValue;
            itP->x = *itX * depthValue;
            itP->y = y * depthValue;
            itP->b = itC->val[0];
            itP->g = itC->val[1];
            itP->r = itC->val[2];
            itP->a = 255;
        }
    }

    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";
    visualizer->addPointCloud(cloud, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setSize(color.cols, color.rows);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);

    while(!visualizer->wasStopped()) {
        visualizer->spinOnce(10);
    }
}

// 创建点云, 缩小一倍 color(1920*1080) depth(1920*1080) cloud(960*540)
void create_point_cloud2(const cv::Mat &color ,const cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                        cv::Mat lookupX, cv::Mat lookupY) {

    const float badPoint = std::numeric_limits<float>::quiet_NaN();

#pragma omp parallel for
    for(int r = 0; r < depth.rows/2; ++r)
    {
        pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols/2];
        const auto *itD = depth.ptr<uint16_t>(r*2);
        const auto *itC = color.ptr<cv::Vec3b>(r*2);
        const float y = lookupY.at<float>(0, r*2);
        const float *itX = lookupX.ptr<float>();

        for(int c = 0; c < depth.cols/2; ++c, ++itP, itD+=2, itC+=2, itX+=2)
        {
//            std::cout << "depth: " << depth.at<uint16_t >(r, c) << std::endl;

            const float depthValue = (float)*itD/ 1000.0f;
//            std::cout << "depthValue: " << depthValue << std::endl;

            // Check for invalid measurements
            if(*itD == 0 || *itD > 1000)
            {
                // not valid
                itP->x = itP->y = itP->z = badPoint;
                itP->rgba = 0;
                continue;
            }
            itP->z = depthValue;
            itP->x = *itX * depthValue;
            itP->y = y * depthValue;
            itP->b = itC->val[0];
            itP->g = itC->val[1];
            itP->r = itC->val[2];
            itP->a = 255;
        }
    }
}

int main(int argc, char** argv)
{
    /// 创建点云
    cv::Mat lookupX, lookupY;
    cv::Mat color, depth;
    pcl::PCDWriter writer;

    // realsense 640*480
//    color = cv::imread("/home/sdhm/03_color_0727.jpg");
//    depth = cv::imread("/home/sdhm/03_depth_0727.png", -1);

    // kinect 960*540
//    color = cv::imread("../data/0001_qhd_color.jpg");
//    depth = cv::imread("../data/0001_qhd_depth.png", -1);

    // cobot kinect 1920*1080
//    color = cv::imread("../data/rgb.jpg");
//    depth = cv::imread("../data/depth.png", -1);

    color = cv::imread("../data/frame-000001.color.jpg");
    depth = cv::imread("../data/frame-000001.depth.png", -1);

    std::cout << "color height " << color.rows << std::endl;
    std::cout << "color width " << color.cols << std::endl;
//    std::cout << depth << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>); // PCL格式点云
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    std::cout << "height " << cloud->height << std::endl;
    std::cout << "width " << cloud->width << std::endl;

    createLookup(lookupX, lookupY, color.cols, color.rows);
    create_point_cloud(color, depth, cloud, lookupX, lookupY);

    // 保存
    writer.writeBinary("../data/cloud.pcd", *cloud);

    return 0;
}

int main1(int argc, char** argv)
{
    /// 创建点云
    cv::Mat lookupX, lookupY;
    cv::Mat color, depth;
    pcl::PCDWriter writer;
    color = cv::imread("/home/hustac/rgb.jpg");
    depth = cv::imread("/home/hustac/depth.png", -1);

    std::cout << "color height " << color.rows << std::endl;
    std::cout << "color width " << color.cols << std::endl;
//    std::cout << depth << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>); // PCL格式点云
    // 点云长宽缩小一倍
    cloud->height = color.rows/2;
    cloud->width = color.cols/2;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    std::cout << "height " << cloud->height << std::endl;
    std::cout << "width " << cloud->width << std::endl;
    std::cout << cloud->points.size() << std::endl;

    createLookup(lookupX, lookupY, cloud->width*2, cloud->height*2);
    create_point_cloud2(color, depth, cloud, lookupX, lookupY);

    // 保存
    writer.writeBinary("/home/hustac/cloud.pcd", *cloud);

    return 0;
}




