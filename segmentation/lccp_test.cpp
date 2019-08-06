//
// Created by sdhm on 5/31/19.
//

#include <stdlib.h>
#include <cmath>
#include <limits.h>
#include <boost/format.hpp>

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <pcl/segmentation/lccp_segmentation.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

int main(int argc, char ** argv)
{
    //输入点云
    pcl::PointCloud<PointT>::Ptr input_cloud_ptr(new pcl::PointCloud<PointT>);
    pcl::PCLPointCloud2 input_pointcloud2;
    if (pcl::io::loadPCDFile("/home/sdhm/Projects/kinect2_cloud_samples/0005_cloud.pcd", input_pointcloud2))
    {
        PCL_ERROR("ERROR: Could not read input point cloud ");
        return (3);
    }
    pcl::fromPCLPointCloud2(input_pointcloud2, *input_cloud_ptr);
    PCL_INFO("Done making cloud\n");

    //超体聚类 参数依次是粒子距离、晶核距离、颜色容差、
    float voxel_resolution = 0.0075f;
    float seed_resolution = 0.03f;
    float color_importance = 0.0f;
    float spatial_importance = 1.0f;
    float normal_importance = 4.0f;
    bool use_single_cam_transform = false;
    bool use_supervoxel_refinement = false;

    unsigned int k_factor = 0;

    //voxel_resolution is the resolution (in meters) of voxels used、seed_resolution is the average size (in meters) of resulting supervoxels
    pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform(use_single_cam_transform);
    super.setInputCloud(input_cloud_ptr);
    //Set the importance of color for supervoxels.
    super.setColorImportance(color_importance);
    //Set the importance of spatial distance for supervoxels.
    super.setSpatialImportance(spatial_importance);
    //Set the importance of scalar normal product for supervoxels.
    super.setNormalImportance(normal_importance);
    std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

    PCL_INFO("Extracting supervoxels\n");
    super.extract(supervoxel_clusters);

    PCL_INFO("Getting supervoxel adjacency\n");
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);
    pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud(supervoxel_clusters);

    //LCCP分割
    float concavity_tolerance_threshold = 10;
    float smoothness_threshold = 0.1;
    uint32_t min_segment_size = 0;
    bool use_extended_convexity = false;
    bool use_sanity_criterion = false;
    PCL_INFO("Starting Segmentation\n");
    pcl::LCCPSegmentation<PointT> lccp;
    lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
    lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
    lccp.setKFactor(k_factor);
    lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
    lccp.setMinSegmentSize(min_segment_size);
    lccp.segment();

    PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");

    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
    lccp.relabelCloud(*lccp_labeled_cloud);
    SuperVoxelAdjacencyList sv_adjacency_list;
    lccp.getSVAdjacencyList(sv_adjacency_list);


    return 0;
}


