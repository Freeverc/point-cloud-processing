#include <cstdlib>
#include <ctime>
#include <iostream>

#include "measure.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char *argv[]) {
  std::cout << "Point cloud measuring : " << std::endl;
  std::cout << "./point_cluster measuring or merge : "
               "(measure, merge)"
            << std::endl;
  std::string point_cloud_path = "../data/03.ply";
  std::string sampled_path = "../results/sampled.pcd";
  std::string filtered_path = "../results/filtered.pcd";
  std::string inner_path = "../results/inner.pcd";

  if (argc == 1) {
    std::cout << "No input file ! " << std::endl;
  } else if (argc == 2) {
    point_cloud_path = argv[1];
    std::cout << "Input : " << point_cloud_path << std::endl;
  } else {
    std::cout << "Too many output! " << std::endl;
  }

  // Read point cloud.
  pcl::PointCloud<PointT>::Ptr point_cloud(new pcl::PointCloud<PointT>);
  if (pcl::io::loadPLYFile<PointT>(point_cloud_path, *point_cloud) == -1) {
    PCL_ERROR("Couldn't read file\n");
    return (-1);
  }
  std::cout << "Size : " << point_cloud->width << " " << point_cloud->height
            << std::endl;

  // Do sampling.
  pcl::PointCloud<PointT>::Ptr sampled_point_cloud(new pcl::PointCloud<PointT>);
  down_sample(point_cloud, sampled_point_cloud);
  std::cout << "Down sampled : " << sampled_point_cloud->size() << std::endl;
  pcl::io::savePCDFileASCII(sampled_path, *sampled_point_cloud);

  // Do filtering.
  pcl::PointCloud<PointT>::Ptr filtered_point_cloud(
      new pcl::PointCloud<PointT>);
  filter(sampled_point_cloud, filtered_point_cloud);
  std::cout << "Filtered : " << filtered_point_cloud->size() << std::endl;
  pcl::io::savePCDFileASCII(filtered_path, *filtered_point_cloud);

  // Do plane detecting.
  pcl::PointCloud<PointT>::Ptr inner_point_cloud(new pcl::PointCloud<PointT>);
  plane_detection(filtered_point_cloud, inner_point_cloud);
  std::cout << "Inner : " << inner_point_cloud->size() << std::endl;
  pcl::io::savePCDFileASCII(inner_path, *inner_point_cloud);
  // Visualizing.
  //   pcl::visualization::CloudViewer viewer("Cloud Viewer");
  //   viewer.showCloud(filtered_point_cloud);
  //   while (!viewer.wasStopped()) {
  //   }
}
