#include "cluster.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char *argv[]) {
  std::cout << "Point cloud clustering : " << std::endl;
  std::cout << "./point_cluster point_cloud_path clustered_path "
               "cluster_method(eu, sv, km)"
            << std::endl;
  std::string point_cloud_path = "../../data/data1.pcd";
  std::string filtered_path = "../../results/filtered.pcd";
  std::string clustered_path = "../../results/clustered.pcd";
  std::string method = "eu";
  if (argc == 1) {
    std::cout << "No input file ! " << std::endl;
  } else if (argc == 2) {
    point_cloud_path = argv[1];
    std::cout << "Input : " << point_cloud_path << std::endl;
  } else if (argc == 3) {
    point_cloud_path = argv[1];
    clustered_path = argv[2];
    std::cout << "Input : " << point_cloud_path << std::endl;
    std::cout << "Output : " << clustered_path << std::endl;
  } else if (argc >= 4) {
    point_cloud_path = argv[1];
    clustered_path = argv[2];
    method = argv[3];
    std::cout << "Input : " << point_cloud_path << std::endl;
    std::cout << "Output : " << clustered_path << std::endl;
  }

  // Read point cloud.
  pcl::PointCloud<PointT>::Ptr point_cloud(new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT>(point_cloud_path, *point_cloud) == -1) {
    PCL_ERROR("Couldn't read file\n");
    return (-1);
  }
  // Do clustering.
  pcl::PointCloud<PointLT>::Ptr clustered_point_cloud(
      new pcl::PointCloud<PointLT>);
  std::cout << "Size : " << point_cloud->width << " " << point_cloud->height
            << std::endl;

  if (method == "eu") {
    cluster_by_euclidean(point_cloud, clustered_point_cloud);
  } else if (method == "sv") {
    cluster_by_super_voxel(point_cloud, clustered_point_cloud);
  } else if (method == "rg") {
    cluster_by_region_growth(point_cloud, clustered_point_cloud);
  } else if (method == "km") {
    if (argc == 5) {
      int k = atoi(argv[4]);
      cluster_by_kmeans(point_cloud, clustered_point_cloud, k);
    } else {
      cluster_by_kmeans(point_cloud, clustered_point_cloud);
    }
  }

  // Write point cloud.
  pcl::io::savePCDFileASCII(clustered_path, *clustered_point_cloud);

  //pcl::visualization::PCLVisualizer::Ptr viewer(
  //    new pcl::visualization::PCLVisualizer("3D Viewer"));
  //viewer->setBackgroundColor(0, 0, 0);

  //viewer->addPointCloud(clustered_point_cloud, "clustered point cloud");

  //while (!viewer->wasStopped()) {
  //  viewer->spinOnce(100);
  //}
  return (0);
}