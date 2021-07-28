#include <iostream>
#include<ctime>
#include<cstdlib>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>

int main()
{
    std::cout<<"testing"<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("../four_add_filter4.pcd", *cloud) == -1) {
		PCL_ERROR("read false");
		return 0;
	}
    std::cout<<cloud->size()<<std::endl;

	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	//showCloud函数是同步的，再此处等待直到渲染显示为止。
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
		//在此处可以添加其他操作。
	}

    //被分割出来的点云团（标号队列）
    std::vector<pcl::PointIndices> cluster_indices;
    //欧式分割器
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    //搜索策略树
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

for (std::vector::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
{
pcl::PointCloud::Ptr cloud_cluster (new pcl::PointCloud);
//创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
for (std::vector::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
cloud_cluster->width = cloud_cluster->points.size ();
cloud_cluster->height = 1;
cloud_cluster->is_dense = true;
}