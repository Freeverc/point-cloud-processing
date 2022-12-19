#include <string>
#include <vector>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/don.h>

using namespace pcl;

int main(int argc, char *argv[]) {
  /// The smallest scale to use in the DoN filter.
  double scale1;

  /// The largest scale to use in the DoN filter.
  double scale2;

  /// The minimum DoN magnitude to threshold by
  double threshold;

  /// segment scene into clusters with given distance tolerance using euclidean
  /// clustering
  double segradius;

  if (argc < 6) {
    std::cerr << "usage: " << argv[0]
              << " inputfile smallscale largescale threshold segradius"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  /// the file to read from.
  std::string infile = argv[1];
  /// small scale
  std::istringstream(argv[2]) >> scale1;
  /// large scale
  std::istringstream(argv[3]) >> scale2;
  std::istringstream(argv[4]) >> threshold; // threshold for DoN magnitude
  std::istringstream(argv[5]) >> segradius; // threshold for radius segmentation

  // 加载点云数据
  pcl::PCLPointCloud2 blob;
  pcl::io::loadPCDFile(infile.c_str(), blob);
  pcl::PointCloud<PointXYZRGB>::Ptr cloud(new pcl::PointCloud<PointXYZRGB>);
  pcl::fromPCLPointCloud2(blob, *cloud);

  // 用树结构组织，加速搜索
  // Create a search tree, use KDTreee for non-organized data.
  pcl::search::Search<PointXYZRGB>::Ptr tree;
  if (cloud->isOrganized()) {
    tree.reset(new pcl::search::OrganizedNeighbor<PointXYZRGB>());
  } else {
    tree.reset(new pcl::search::KdTree<PointXYZRGB>(false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud(cloud);

  if (scale1 >= scale2) {
    std::cerr << "Error: Large scale must be > small scale!" << std::endl;
    exit(EXIT_FAILURE);
  }

  // 计算法向量
  // Compute normals using both small and large scales at each point
  pcl::NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);

  /**
   * NOTE: setting viewpoint is very important, so that we can ensure
   * normals are all pointed in the same direction!
   */
  ne.setViewPoint(std::numeric_limits<float>::max(),
                  std::numeric_limits<float>::max(),
                  std::numeric_limits<float>::max());

  // 计算小半径的法向量
  // calculate normals with the small scale
  std::cout << "Calculating normals for scale..." << scale1 << std::endl;
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale(
      new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch(scale1);
  ne.compute(*normals_small_scale);

  // 计算大半径的法向量
  // calculate normals with the large scale
  std::cout << "Calculating normals for scale..." << scale2 << std::endl;
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale(
      new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch(scale2);
  ne.compute(*normals_large_scale);

  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud(new pcl::PointCloud<PointNormal>);
  copyPointCloud(*cloud, *doncloud);

  std::cout << "Calculating DoN... " << std::endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
  don.setInputCloud(cloud);
  don.setNormalScaleLarge(normals_large_scale);
  don.setNormalScaleSmall(normals_small_scale);

  if (!don.initCompute()) {
    std::cerr << "Error: Could not initialize DoN feature operator"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  // Compute DoN
  don.computeFeature(*doncloud);

  // Save DoN features
  pcl::PCDWriter writer;
  writer.write<pcl::PointNormal>("don.pcd", *doncloud, false);

  // Filter by magnitude
  std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;

  // Build the condition for filtering
  pcl::ConditionOr<PointNormal>::Ptr range_cond(
      new pcl::ConditionOr<PointNormal>());
  range_cond->addComparison(pcl::FieldComparison<PointNormal>::ConstPtr(
      new pcl::FieldComparison<PointNormal>("curvature", pcl::ComparisonOps::GT,
                                            threshold)));
  // Build the filter
  pcl::ConditionalRemoval<PointNormal> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(doncloud);

  pcl::PointCloud<PointNormal>::Ptr doncloud_filtered(
      new pcl::PointCloud<PointNormal>);

  // Apply filter
  condrem.filter(*doncloud_filtered);

  doncloud = doncloud_filtered;

  // Save filtered output
  std::cout << "Filtered Pointcloud: " << doncloud->size() << " data points."
            << std::endl;

  writer.write<pcl::PointNormal>("don_filtered.pcd", *doncloud, false);

  // std::vector<pcl::PointIndices> cluster_indices(threshold_list.size());

  // // print some differencense of curvature
  // std::vector<double> threshold_list = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8,
  // 0.9};
  // {
  //   std::cout
  //       << "You may have some sense about the input threshold（curvature） "
  //          "next time for your data"
  //       << std::endl;
  //   int size_cloud = doncloud->size();
  //   for (int i = 0; i < size_cloud; i++) {
  //     if (i % 1000 == 0) {
  //       std::cout << doncloud->points[i].curvature << std::endl;
  //     }

  //     for (int j = 0; j < threshold_list.size(); ++j) {
  //       if (doncloud->points[i].curvature < threshold_list[j]) {
  //         cluster_indices[j].indices.push_back(i);
  //         break;
  //       }
  //     }
  //   }
  // }

  std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= "
            << segradius << "..." << std::endl;

  pcl::search::KdTree<PointNormal>::Ptr segtree(
      new pcl::search::KdTree<PointNormal>);
  segtree->setInputCloud(doncloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointNormal> ec;

  ec.setClusterTolerance(segradius);
  ec.setMinClusterSize(50);
  ec.setMaxClusterSize(100000);
  ec.setSearchMethod(segtree);
  ec.setInputCloud(doncloud);
  ec.extract(cluster_indices);

  // 输出
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it, j++) {
    pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don(
        new pcl::PointCloud<PointNormal>);
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      cloud_cluster_don->points.push_back((*doncloud)[*pit]);
    }

    cloud_cluster_don->width = cloud_cluster_don->size();
    cloud_cluster_don->height = 1;
    cloud_cluster_don->is_dense = true;

    // Save cluster
    std::cout << "PointCloud representing the Cluster: "
              << cloud_cluster_don->size() << " data points." << std::endl;
    if (cloud_cluster_don->size() <= 0) {
      continue;
    }

    std::stringstream ss;
    ss << "don_cluster_" << j << ".pcd";
    writer.write<pcl::PointNormal>(ss.str(), *cloud_cluster_don, false);
  }
}