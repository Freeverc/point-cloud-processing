#include <float.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <pcl/features/don.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

// Types
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

void cluster_by_euclidean(pcl::PointCloud<PointT>::Ptr &point_cloud,
                          pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud);

void cluster_by_super_voxel(
    pcl::PointCloud<PointT>::Ptr &point_cloud,
    pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud);

void cluster_by_region_growth(
    pcl::PointCloud<PointT>::Ptr &point_cloud,
    pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud);

void cluster_by_kmeans(pcl::PointCloud<PointT>::Ptr &point_cloud,
                       pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud,
                       int k = 4);

void cluster_by_lccp(pcl::PointCloud<PointT>::Ptr &point_cloud,
                     pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud);

void IndiceToClustered(pcl::PointCloud<PointT>::Ptr &point_cloud,
                       std::vector<pcl::PointIndices> &cluster_indices,
                       pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud);

void cluster_by_normal(pcl::PointCloud<PointT>::Ptr &point_cloud,
                       pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud,
                       int k = 4);