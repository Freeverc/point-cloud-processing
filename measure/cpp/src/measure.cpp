#include "measure.h"

void down_sample(pcl::PointCloud<PointT>::Ptr &point_cloud,
                 pcl::PointCloud<PointT>::Ptr &sampled_point_cloud) {
  std::cout << "clustering by euclid..." << std::endl;

  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(point_cloud);
  vg.setLeafSize(0.05f, 0.05f, 0.05f);
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
  sor.setStddevMulThresh(1);
  sor.filter(*filtered_point_cloud);
  std::cout << "PointCloud after filtering has: "
            << filtered_point_cloud->size() << " data points." << std::endl;
}

void plane_detection(pcl::PointCloud<PointT>::Ptr &point_cloud,
                     pcl::PointCloud<PointT>::Ptr &inner_point_cloud) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.2);

  seg.setInputCloud(point_cloud);
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
  pcl::io::savePLYFileASCII("../results/plane.ply", *inner_point_cloud);

  Eigen::Matrix4f rotate_x = Eigen::Matrix4f::Identity();
  float theta_x = std::acos(c / std::sqrt(c * c + b * b));
  rotate_x(1, 1) = std::cos(theta_x);
  rotate_x(1, 2) = std::sin(theta_x);
  rotate_x(2, 1) = -std::sin(theta_x);
  rotate_x(2, 2) = std::cos(theta_x);
  std::cout << "rotate x : " << std::endl;
  std::cout << rotate_x << std::endl;
  pcl::PointCloud<PointT>::Ptr transformed_x_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*inner_point_cloud, *transformed_x_cloud, rotate_x);
  pcl::io::savePLYFileASCII("../results/transformed_x.ply",
                            *transformed_x_cloud);

  Eigen::Matrix4f rotate_y = Eigen::Matrix4f::Identity();
  float theta_y = std::acos(c / std::sqrt(c * c + a * a));
  rotate_y(0, 0) = std::cos(theta_y);
  rotate_y(0, 2) = -std::sin(theta_y);
  rotate_y(2, 0) = std::sin(theta_y);
  rotate_y(2, 2) = std::cos(theta_y);
  std::cout << "rotate y : " << std::endl;
  std::cout << rotate_y << std::endl;
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>());
  pcl::transformPointCloud(*transformed_x_cloud, *transformed_cloud, rotate_y);
  pcl::io::savePLYFileASCII("../results/transformed.ply", *transformed_cloud);

  float min_x, min_y, min_z, max_x, max_y, max_z;
  min_x = min_y = min_z = FLT_MAX;
  max_x = max_y = max_z = FLT_MIN;
  for (int i = 0; i < transformed_cloud->size(); ++i) {
    float x = (*inner_point_cloud)[i].x;
    float y = (*inner_point_cloud)[i].y;
    float z = (*inner_point_cloud)[i].z;
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
  std::cout << "inner cloud : " << std::endl;
  std::cout << "Min : " << min_x << " " << min_y << " " << min_z << std::endl;
  std::cout << "Max : " << max_x << " " << max_y << " " << max_z << std::endl;
  min_x = min_y = min_z = FLT_MAX;
  max_x = max_y = max_z = FLT_MIN;
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
  float resolution = 0.1;
  int size_x = (max_x - min_x) / resolution + 1;
  int size_y = (max_y - min_y) / resolution + 1;
  cv::Mat image(size_x, size_y, CV_8UC1, cv::Scalar(0));
  std::vector<cv::Vec3f> circles;

  for (int i = 0; i < transformed_cloud->size(); ++i) {
    float x = (*transformed_cloud)[i].x;
    float y = (*transformed_cloud)[i].y;
    int x_i = std::floor((x - min_x) / resolution);
    int y_i = std::floor((y - min_y) / resolution);
    // std::cout << x_i << " " << y_i << std::endl;
    image.at<uint8_t>(x_i, y_i) = 255;
  }
  std::cout << "image generated : " << std::endl;

  cv::imwrite("../results/projection.jpg", image);
  HoughCircles(image, circles, cv::HOUGH_GRADIENT, 1, 80, 20, 10.5, 20, 100);

  cv::Mat result(size_x, size_y, CV_8UC1, cv::Scalar(0));
  for (size_t i = 0; i < circles.size(); i++) {
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    //检测圆中心
    cv::circle(result, center, 3, cv::Scalar(255), -1, 8, 0);
    //检测圆轮廓
    cv::circle(result, center, radius, cv::Scalar(255), 1, 8, 0);
    float x = min_x + center.x * resolution + resolution / 2;
    float y = min_y + center.y * resolution + resolution / 2;
    float r = radius * resolution + resolution / 2;
    std::cout << "x: " << x << " y : " << y << " r : " << r << std::endl;
  }
  cv::imwrite("../results/circles.jpg", result);
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