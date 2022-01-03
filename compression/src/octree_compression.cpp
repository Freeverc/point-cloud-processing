#define _SILENCE_FPOS_SEEKPOS_DEPRECATION_WARNING
#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

void compress_by_oc(std::string& point_cloud_path, std::string& compressed_path) {
	// Read point cloud 输入点云文件
	pcl::PointCloud<pcl::PointXYZRGB> sourceCloud;
	pcl::PLYReader reader;
	if (pcl::io::loadPLYFile(point_cloud_path, sourceCloud) == -1) {
		PCL_ERROR("Failed to load PLYFile!");
		return;
	}

	bool showStatistics = true;
	pcl::io::compression_Profiles_e compressionProfile = pcl::io::MANUAL_CONFIGURATION;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
	PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compressionProfile, showStatistics, 0.001, 0.01,
		true, 100, true, 8);//输入参数
	std::stringstream compressedData;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());
	PointCloudEncoder->encodePointCloud(sourceCloud.makeShared(), compressedData);
	std::ofstream compressed_file(compressed_path, std::fstream::binary | std::fstream::out);
	compressed_file << compressedData.str();
	compressed_file.close();

}

int main(int argc, char* argv[])
{
	std::cout << "Point cloud compression by octree : " << std::endl;
	std::cout << "./octree_compression point_cloud_path compressed_path "
		<< std::endl;
	std::string point_cloud_path = "C:/Users/Freeverc/Projects/point-cloud-processing/data/compression/longdress_vox10_1051.ply";
	std::string compressed_path = "C:/Users/Freeverc/Projects/point-cloud-processing/data/compression/compressed_octree.ot";
	std::string method = "oc";
	if (argc == 1) {
		std::cout << "No input file ! " << std::endl;
	}
	else if (argc == 2) {
		point_cloud_path= argv[1];
	}
	else if (argc == 3) {
		point_cloud_path = argv[1];
		compressed_path= argv[2];
	}
	else if (argc >= 4) {
		std::cout << "Too many input file ! " << std::endl;
	}

	compress_by_oc(point_cloud_path, compressed_path);
	std::cout << "Compressed : " << compressed_path << std::endl;

	return 0;
}
