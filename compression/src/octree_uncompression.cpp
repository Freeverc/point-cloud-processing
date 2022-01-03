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

void uncompress_by_oc(std::string& compressed_path, std::string& uncompressed_path) {
	bool showStatistics = true;
	pcl::io::compression_Profiles_e compressionProfile = pcl::io::MANUAL_CONFIGURATION;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
	PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compressionProfile, showStatistics, 0.001, 0.01,
		true, 100, true, 8);//输入参数
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());
	std::ifstream bitstreamFile(compressed_path, std::ios::binary);
	//PointCloudEncoder->decodePointCloud(compressedData, cloudOut);//由实时流解码
	PointCloudEncoder->decodePointCloud(bitstreamFile, cloudOut);//由bin文件解码
	pcl::PLYWriter writer;
	writer.write(uncompressed_path, *cloudOut, false, false);
}

int main(int argc, char* argv[])
{
	std::cout << "Point cloud uncompression by octree : " << std::endl;
	std::string compressed_path = "C:/Users/Freeverc/Projects/point-cloud-processing/data/compression/compressed_octree.ot";
	std::string uncompressed_path = "C:/Users/Freeverc/Projects/point-cloud-processing/data/compression/uncompressed_octree.ply";
	std::string method = "oc";
	if (argc == 1) {
		std::cout << "No input file ! " << std::endl;
	}
	else if (argc == 2) {
		compressed_path= argv[1];
	}
	else if (argc == 3) {
		compressed_path= argv[1];
		uncompressed_path = argv[2];
	}
	else if (argc >= 4) {
		std::cout << "Too many input file ! " << std::endl;
	}

	std::cout << "Input : " <<compressed_path << std::endl;
	uncompress_by_oc(compressed_path, uncompressed_path);
	std::cout << "Uncompressed : " << uncompressed_path << std::endl;
	return 0;
}
