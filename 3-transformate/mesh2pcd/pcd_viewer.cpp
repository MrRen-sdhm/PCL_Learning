#include <iostream> //标准输入输出流
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件

int main(int argc, char** argv)
{
    // Parse the command line arguments for PCD file
    std::vector<int> pcd_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    if (pcd_file_indices.size () != 1)
    {
        pcl::console::print_error ("Need a single input PCD file to continue.\n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // 创建点云（指针）

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[pcd_file_indices[0]], *cloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
    {
        PCL_ERROR("Couldn't read file !\n"); // 文件不存在时，返回错误，终止程序
        return (-1);
    }

    pcl::visualization::PCLVisualizer viewer ("PCD Viewer");
    viewer.setBackgroundColor (0.15, 0.15, 0.15);
    viewer.addPointCloud (cloud);
    viewer.addCoordinateSystem (0.1);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return (0);

}

