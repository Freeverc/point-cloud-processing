#include "measure.h"

void down_sample(pcl::PointCloud<PointT>::Ptr &point_cloud,
                 pcl::PointCloud<PointT>::Ptr &sampled_point_cloud) {
  std::cout << "clustering by euclid..." << std::endl;

  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(point_cloud);
  vg.setLeafSize(0.001f, 0.001f, 0.001f);
  vg.filter(*sampled_point_cloud);
  std::cout << "PointCloud after sampling has: " << sampled_point_cloud->size()
            << " data points." << std::endl; //*
}

void filter(pcl::PointCloud<PointT>::Ptr &point_cloud,
            pcl::PointCloud<PointT>::Ptr &filtered_point_cloud) {
  // Filtering.
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(point_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(3);
  sor.filter(*filtered_point_cloud);
  std::cout << "PointCloud after filtering has: "
            << filtered_point_cloud->size() << " data points." << std::endl;
}

void plane_detection(pcl::PointCloud<PointT>::Ptr &point_cloud,
                     pcl::PointCloud<PointT>::Ptr &inner_point_cloud, float resolution) {
  float min_x, min_y, min_z, max_x, max_y, max_z;
  min_x = min_y = min_z = FLT_MAX;
  max_x = max_y = max_z = -FLT_MAX;
  for (int i = 0; i < point_cloud->size(); ++i) {
    float x = (*point_cloud)[i].x;
    float y = (*point_cloud)[i].y;
    float z = (*point_cloud)[i].z;
    if (x < min_x)
      min_x = x;
    if (y < min_y)
     min_y = y;
    if (z < min_z)
      min_z = z;
    if (x > max_x)
      max_x = x;
    if (y > max_y)
      max_y = y;
    if (z > max_z)
      max_z = z;
  }


  Eigen::Affine3d trans= Eigen::Affine3d::Identity();
  Eigen::Vector4f cent;
  pcl::compute3DCentroid(*point_cloud, cent);
  trans.translation() << -cent[0], -cent[1], -cent[2];
  std::cout << "Init point cloud : " << std::endl;
  std::cout << "centroid : " << cent<<  std::endl;
  std::cout << "Min : " << min_x << " " << min_y << " " << min_z << std::endl;
  std::cout << "Max : " << max_x << " " << max_y << " " << max_z << std::endl;

  pcl::PointCloud<PointT>::Ptr moved_cloud(new pcl::PointCloud<PointT>());
  pcl::transformPointCloud(*point_cloud, *moved_cloud, trans);
  pcl::io::savePLYFileASCII("../moved_cloud.ply", *moved_cloud);

  // Create the segmentation object
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.04);

  seg.setInputCloud(moved_cloud);
  seg.segment(*inliers, *coefficients);

  for (std::vector<int>::const_iterator pit = inliers->indices.begin();
       pit != inliers->indices.end(); ++pit) {
    PointT p;
    p.x = (*point_cloud)[*pit].x;
    p.y = (*point_cloud)[*pit].y;
    p.z = (*point_cloud)[*pit].z;
    inner_point_cloud->push_back(p);
  }

  float a = coefficients->values[0];
  float b = coefficients->values[1];
  float c = coefficients->values[2];
  float d = coefficients->values[3];

  std::cout << "Inner num : " << inner_point_cloud->size() << std::endl;
  std::cout << "Model coefficients: " << a << " " << b << " " << c << " " << d
            << std::endl;

  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  Eigen::Vector3d vectorBefore(a, b, c);
  Eigen::Vector3d vectorAfter(0, 0, 1);
  Eigen::Matrix3d rotMatrix;
  rotMatrix = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter).toRotationMatrix();
  transform.rotate(rotMatrix);
  std::cout << "rotation : " << std::endl;
  std::cout << rotMatrix<< std::endl;

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*moved_cloud, centroid);
  transform.translation() << -centroid[0], -centroid[1], -centroid[2];
  std::cout << "centroid : " << centroid <<  std::endl;

  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>());
  pcl::transformPointCloud(*moved_cloud, *transformed_cloud, transform);
  pcl::io::savePLYFileASCII("../transformed.ply", *transformed_cloud);

  min_x = min_y = min_z = FLT_MAX;
  max_x = max_y = max_z = -FLT_MAX;
  for (int i = 0; i < transformed_cloud->size(); ++i) {
    float x = (*transformed_cloud)[i].x;
    float y = (*transformed_cloud)[i].y;
    float z = (*transformed_cloud)[i].z;
    if (x < min_x)
      min_x = x;
    if (y < min_y)
      min_y = y;
    if (z < min_z)
      min_z = z;
    if (x > max_x)
      max_x = x;
    if (y > max_y)
      max_y = y;
    if (z > max_z)
      max_z = z;
  }

  std::cout << "transformed cloud : " << std::endl;
  std::cout << "Min : " << min_x << " " << min_y << " " << min_z << std::endl;
  std::cout << "Max : " << max_x << " " << max_y << " " << max_z << std::endl;

  // Generate image.
  
  int size_x = (max_x - min_x) / resolution + 1;
  int size_y = (max_y - min_y) / resolution + 1;
  std::cout << "Size : " << size_x << " " << size_y <<  std::endl;
  cv::Mat image(size_x, size_y, CV_8UC1, cv::Scalar(0));
  std::vector<cv::Vec3f> circles;

  for (int i = 0; i < transformed_cloud->size(); ++i) {
    float x = (*transformed_cloud)[i].x;
    float y = (*transformed_cloud)[i].y;
    float z = (*transformed_cloud)[i].z;
    int x_i = std::floor((x - min_x) / resolution);
    int y_i = std::floor((y - min_y) / resolution);
    float c = 255 * (z - min_z)/ (max_z - min_z);
    if(c > image.at<uint8_t>(x_i, y_i))
    {
      image.at<uint8_t>(x_i, y_i) = c;
    }
  }

  std::cout << "image generated : " << std::endl;
  cv::imwrite("../projection.jpg", image);
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