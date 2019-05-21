#include <iostream>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;
int main()
{
	/*+++++++++++++++++++++++++单视角点云获取+++++++++++++++++++++++++++++++*/
	string name = "rectangle";
    string stlName = name + ".STL";

	//读取CAD模型
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(stlName.c_str());
	reader->Update();
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata = reader->GetOutput();
	polydata->GetNumberOfPoints();

	//***单视角点云获取
	//主要是renderViewTesselatedSphere的参数设定
	//输入
	float resx = 255;   //显示视点图窗的X大小  分辨率，值越大，采集的点越多
	float resy = resx;  //显示视点图窗的Y大小

	std::vector<pcl::PointCloud<pcl::PointXYZ>,
		Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > views_xyz; // 各视点点云对应的XYZ信息

//	pcl::PointCloud<pcl::PointXYZ>::CloudVectorType views_xyz;

	//输出
	std::vector<Eigen::Matrix4f,
		Eigen::aligned_allocator<Eigen::Matrix4f> > poses; // 从目标坐标变换到视点相机坐标
	std::vector<float> entropies; //0-1之间，视点看到模型的百分比

	//输入
	int tesselation_level = 1; //表示在角度下的细分数
	float view_angle = 45; //虚拟相机的视场
	float radius_sphere = 0.35; //radius_sphere半径, 即相机围绕物体旋转的半径
	bool use_vertices = false; //是否采用tessellated icosahedron 的vertices


    //PCLVisualizer 显示
	pcl::visualization::PCLVisualizer vis ("vis");
	vis.addModelFromPolyData(polydata, "mesh", 0);
	vis.setRepresentationToSurfaceForAllActors();
	vis.renderViewTesselatedSphere(resx, resy, views_xyz, poses, entropies,
		tesselation_level, view_angle, radius_sphere, use_vertices); //显示个角度点云

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr views_cloud_show (new pcl::PointCloud<pcl::PointXYZRGBA>);

	//保存
	for (int i = 0; i < views_xyz.size(); i++)
	{
//		pcl::PointCloud<pcl::PointXYZ> views_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr views_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZRGBA> views_cloud_xyzrgba;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr views_cloud_xyzrgba (new pcl::PointCloud<pcl::PointXYZRGBA>);
//        poses[i] = Eigen::Matrix4f::Identity(); // 不进行坐标转换
		pcl::transformPointCloud(views_xyz[i]/*输入点云*/, *views_cloud/*输出点云*/, poses[i]/*刚性变换*/);
		std::cout << "pose" << i << ":\n" << poses[i] << std::endl;

		// PointXYZ -> PointXYZRGBA
        views_cloud_xyzrgba->points.resize(views_cloud->size());
        printf("views_cloud size: %zu %d*%d\n", views_cloud->size(), views_cloud->width, views_cloud->height);
        for (size_t j = 0; j < views_cloud->points.size(); j++) {
            views_cloud_xyzrgba->points[j].x = views_cloud->points[j].x;
            views_cloud_xyzrgba->points[j].y = views_cloud->points[j].y;
            views_cloud_xyzrgba->points[j].z = views_cloud->points[j].z;
//			views_cloud_xyzrgba->points[j].r = 255;
//			views_cloud_xyzrgba->points[j].g = 255;
			views_cloud_xyzrgba->points[j].b = 255;
        }
        views_cloud_xyzrgba->width = views_cloud->width;
        views_cloud_xyzrgba->height = views_cloud->height;
        printf("views_cloud_xyzrgba size: %zu\n", views_cloud_xyzrgba->size());

		// 保存第一个视角以可视化
		if(i == 0) views_cloud_show = views_cloud_xyzrgba;

//		std::stringstream ss;
//		ss << "cloud_view_" << i << ".ply";
//		pcl::io::savePLYFile(ss.str(), views_cloud);

        std::stringstream ss_pcd;
        ss_pcd << "./pcd/" << name.c_str() << "_view_" << i << ".pcd";
        cout << "Save: ./pcd/" << name.c_str() << "_view_" << i << ".pcd\n";
        pcl::io::savePCDFile(ss_pcd.str(), *views_cloud_xyzrgba);

        std::stringstream ss_pcd_ascii;
        ss_pcd_ascii << "./pcd_ascii/" << name.c_str() << "_view_" << i << ".pcd";
        cout << "Save: ./pcd_ascii/" << name.c_str() << "_view_" << i << ".pcd\n";
        pcl::io::savePCDFileASCII(ss_pcd_ascii.str(), *views_cloud_xyzrgba);
	}

	visualization::PCLVisualizer vis2 ("vis2");
	vis2.setBackgroundColor (1, 0, 0);
	vis2.addCoordinateSystem (0.1);
	vis2.addPointCloud (views_cloud_show);

	//显示原STL文件
	while ( ! vis.wasStopped() && ! vis2.wasStopped())
	{
		vis.spinOnce();
		vis2.spinOnce();
	}
}

/*
renderViewTesselatedSphere函数说明

这个函数是从不同视角得到CAD模型的部分视图。这里设定的视角是一个包在CAD模型外面的，由正三角形组成的二十面体，虚拟的相机从二十面
体的每个顶点（或者每个面）拍摄CAD模型，然后得到对应视角下的点云。这个函数如果不改内部的代码，是不能指定视角的，每次运行，虚拟的
相机会在每个顶点（或者面）都拍一遍，得到12个（或者20个，对应面的数量）视角下的点云，之后可以挑选自己需要的点云来进行之后的操作。

参数如下：

[in] xres       	   窗口x方向大小（即分辨率），分辨率越大，采样点云包含点的数目越多
[in] yres        	   窗口y方向大小（即分辨率），分辨率越大，采样点云包含点的数目越多  
[in] cloud       	   有XYZ信息的点云向量代表各视角下的模型
[out] poses      	   从物体坐标系到各视角相机坐标系的位姿转换
[out] enthropies       在0-1之间，各视角看到模型的比率
[in] tesselation_level     对于原始二十面体三角形面的分割数，如果设为0，则是原始二十面体，设为1，每个三角形面会被分为4个三角形
[in] view_angle        相机的视场角FOV，默认为45  
[in] radius_sphere     半径，默认为1
[in] use_vertices      设为TRUE，则使用顶点，得到12个视角（tesselation_level =0）或42个视角（tesselation_level =1），设为
					   FALSE，则使用面，得到得到20个视角（tesselation_level =0）或80个视角（tesselation_level =1）
*/
