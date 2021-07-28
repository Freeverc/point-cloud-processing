#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <string>
#include <vector>

uint8_t kClusterColors[6][3] = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                                {128, 128, 0}, {128, 0, 128}, {0, 128, 128}};

int main() {
  std::cout << "Hello " << std::endl;

  std::string file_point_cloud = "../data1.pcd";
  std::string filtered_point_cloud = "../filtered.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_point_cloud, *point_cloud) ==
      -1) //* load the file
  {
    PCL_ERROR("Couldn't read file\n");
    return (-1);
  }
  std::cout << "Size : " << point_cloud->width << " " << point_cloud->height
            << std::endl;

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(point_cloud);
  vg.setLeafSize(0.001f, 0.001f, 0.001f);
  vg.filter(*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size()
            << " data points." << std::endl; //*

  pcl::io::savePCDFileASCII(filtered_point_cloud, *cloud_filtered);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(point_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.005); // 2cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(250000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
  std::cout << "cluster num : " << cluster_indices.size() << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      pcl::PointXYZRGB p;
      p.x = (*cloud_filtered)[*pit].x;
      p.y = (*cloud_filtered)[*pit].y;
      p.z = (*cloud_filtered)[*pit].z;
      p.r = kClusterColors[j][0];
      p.g = kClusterColors[j][1];
      p.b = kClusterColors[j][2];
      cloud_cluster->push_back(p);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: "
              << cloud_cluster->size() << " data points." << std::endl;
    j++;
  }

  std::string file_cloud_clustered("../cloud_clustered.pcd");
  pcl::io::savePCDFileASCII(file_cloud_clustered, *cloud_cluster);
}