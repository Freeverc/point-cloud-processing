#include "cluster.h"

void cluster_by_euclidean(
    pcl::PointCloud<PointT>::Ptr &point_cloud,
    pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud) {
  std::cout << "clustering by euclid..." << std::endl;

  pcl::VoxelGrid<PointT> vg;
  pcl::PointCloud<PointT>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(point_cloud);
  vg.setLeafSize(0.001f, 0.001f, 0.001f);
  vg.filter(*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size()
            << " data points." << std::endl; //*

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(point_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(0.005);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(250000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
  std::cout << "cluster num : " << cluster_indices.size() << std::endl;
  IndiceToClustered(cloud_filtered, cluster_indices, clustered_point_cloud);
  std::cout << "PointCloud representing the Cluster: "
            << clustered_point_cloud->size() << " data points." << std::endl;
}

void cluster_by_super_voxel(
    pcl::PointCloud<PointT>::Ptr &point_cloud,
    pcl::PointCloud<PointLT>::Ptr &clustred_point_cloud) {
  // Filtering.
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(point_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(2);
  sor.filter(*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size()
            << " data points." << std::endl;

  // Clustering.
  std::cout << "clustering by voxel..." << std::endl;
  float voxel_resolution = 0.004f;
  float seed_resolution = 0.2f;
  float color_importance = 0;
  float spatial_importance = 0.9f;
  float normal_importance = 0.5f;

  pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
  super.setUseSingleCameraTransform(false);
  super.setInputCloud(cloud_filtered);
  super.setColorImportance(color_importance);
  super.setSpatialImportance(spatial_importance);
  super.setNormalImportance(normal_importance);

  std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
  pcl::console::print_highlight("Extracting supervoxels!\n");
  super.extract(supervoxel_clusters);
  pcl::console::print_info("Found %d supervoxels\n",
                           supervoxel_clusters.size());

  clustred_point_cloud = super.getLabeledVoxelCloud();
}

void cluster_by_region_growth(
    pcl::PointCloud<PointT>::Ptr &point_cloud,
    pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud) {

  // Filtering.
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(point_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(2.0);
  sor.filter(*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size()
            << " data points." << std::endl; //*

  std::string fliter_path = "../../results/filter.pcd";
  pcl::io::savePCDFileASCII(fliter_path, *cloud_filtered);

  std::cout << "clustering by region growth..." << std::endl;
  pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud_filtered);
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*normals);

  pcl::IndicesPtr indices(new std::vector<int>);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*indices);

  pcl::RegionGrowing<PointT, pcl::Normal> reg;
  reg.setMinClusterSize(1);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(30);
  reg.setInputCloud(cloud_filtered);
  // reg.setIndices (indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(25.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1);

  std::vector<pcl::PointIndices> cluster_indices;
  reg.extract(cluster_indices);

  IndiceToClustered(cloud_filtered, cluster_indices, clustered_point_cloud);
  std::cout << "PointCloud representing the Cluster: "
            << clustered_point_cloud->size() << " data points." << std::endl;

  std::cout << "Number of clusters is equal to " << cluster_indices.size()
            << std::endl;
}

void cluster_by_kmeans(pcl::PointCloud<PointT>::Ptr &point_cloud,
                       pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud,
                       int k) {

  std::cout << "Clustering by kmeans..." << std::endl;
  std::vector<double> center(3, 0);
  // Initialize cluster of points.
  srand((unsigned)time(NULL));
  for (int i = 0; i < point_cloud->size(); ++i) {
    PointLT p;
    p.x = (*point_cloud)[i].x;
    p.y = (*point_cloud)[i].y;
    p.z = (*point_cloud)[i].z;
    p.label = rand() % k;
    clustered_point_cloud->push_back(p);
    center[0] += p.x;
    center[1] += p.y;
    center[2] += p.z;
  }
  center[0] /= point_cloud->size();
  center[1] /= point_cloud->size();
  center[2] /= point_cloud->size();

  std::vector<int> cluster_nums(k, 0);
  std::vector<std::vector<double>> cluster_centroids(k,
                                                     std::vector<double>(3, 0));
  bool iterate_stop = false;
  int iterate_id = 0;
  while (!iterate_stop) {
    std::cout << "Iterate id : " << iterate_id << std::endl;
    // Calculate centroid.
    cluster_nums.assign(cluster_nums.size(), 0);
    cluster_centroids.assign(cluster_centroids.size(),
                             std::vector<double>(3, 0));
    for (int i = 0; i < clustered_point_cloud->size(); ++i) {
      float x = (*clustered_point_cloud)[i].x;
      float y = (*clustered_point_cloud)[i].y;
      float z = (*clustered_point_cloud)[i].z;
      unsigned int label = (*clustered_point_cloud)[i].label;
      cluster_centroids[label][0] += x;
      cluster_centroids[label][1] += y;
      cluster_centroids[label][2] += z;
      cluster_nums[label] += 1;
    }
    for (int j = 0; j < k; ++j) {
      if (cluster_nums[j] != 0) {
        cluster_centroids[j][0] /= cluster_nums[j];
        cluster_centroids[j][1] /= cluster_nums[j];
        cluster_centroids[j][2] /= cluster_nums[j];
      } else {
        cluster_centroids[j][0] = center[0];
        cluster_centroids[j][1] = center[1];
        cluster_centroids[j][2] = center[2];
      }
      std::cout << "Cluster " << j << " : " << cluster_nums[j] << " | "
                << cluster_centroids[j][0] << " " << cluster_centroids[j][1]
                << " " << cluster_centroids[j][2] << std::endl;
    }
    // Calculate distance.
    for (int i = 0; i < clustered_point_cloud->size(); ++i) {
      float x = (*clustered_point_cloud)[i].x;
      float y = (*clustered_point_cloud)[i].y;
      float z = (*clustered_point_cloud)[i].z;

      float d_min = FLT_MAX;
      for (int j = 0; j < k; ++j) {
        float x_c = cluster_centroids[j][0];
        float y_c = cluster_centroids[j][1];
        float z_c = cluster_centroids[j][2];
        float d = (x - x_c) * (x - x_c) + (y - y_c) * (y - y_c) +
                  (z - z_c) * (z - z_c);
        if (d < d_min) {
          (*clustered_point_cloud)[i].label = j;
          d_min = d;
        }
      }
    }
    ++iterate_id;
    if (iterate_id > 200) {
      iterate_stop = true;
    }
  }
  std::cout << "Finished clustering by kmeans." << std::endl;
}

void cluster_by_lccp(pcl::PointCloud<PointT>::Ptr &point_cloud,
                     pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud) {
  int a;
}

void IndiceToClustered(pcl::PointCloud<PointT>::Ptr &point_cloud,
                       std::vector<pcl::PointIndices> &cluster_indices,
                       pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud) {
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      PointLT p;
      p.x = (*point_cloud)[*pit].x;
      p.y = (*point_cloud)[*pit].y;
      p.z = (*point_cloud)[*pit].z;
      p.label = j;
      clustered_point_cloud->push_back(p);
    }
    j++;
  }
  clustered_point_cloud->width = clustered_point_cloud->size();
  clustered_point_cloud->height = 1;
  clustered_point_cloud->is_dense = true;
}

void cluster_by_normal(pcl::PointCloud<PointT>::Ptr &point_cloud,
                       pcl::PointCloud<PointLT>::Ptr &clustered_point_cloud,
                       int k) {
  // Create a search tree, use KDTreee for non-organized data.
  pcl::search::Search<PointT>::Ptr tree;
  if (point_cloud->isOrganized()) {
    tree.reset(new pcl::search::OrganizedNeighbor<PointT>());
  } else {
    tree.reset(new pcl::search::KdTree<PointT>(false));
  }

  tree->setInputCloud(point_cloud);
  ;
}