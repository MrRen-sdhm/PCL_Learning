#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;


int
main ()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    // load the file
    if (pcl::io::loadPCDFile("test.pcd", *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read pcd file\n");
        return (-1);
    }

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    viewer.showCloud(cloud);

    while (!viewer.wasStopped ())
    {

    }
    return 0;
}
