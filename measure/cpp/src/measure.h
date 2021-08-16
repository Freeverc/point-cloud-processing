#include <float.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

// Types
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

void down_sample(pcl::PointCloud<PointT>::Ptr &point_cloud,
                 pcl::PointCloud<PointT>::Ptr &sampled_point_cloud);

void filter(pcl::PointCloud<PointT>::Ptr &point_cloud,
            pcl::PointCloud<PointT>::Ptr &filtered_point_cloud);

void plane_detection(pcl::PointCloud<PointT>::Ptr &point_cloud,
                     pcl::PointCloud<PointT>::Ptr &inner_point_cloud);

void IndiceToClustered(pcl::PointCloud<PointT>::Ptr &point_cloud,
                       std::vector<pcl::PointIndices> &cluster_indices,
                       pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud);