#define _SILENCE_FPOS_SEEKPOS_DEPRECATION_WARNING

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

void compress_by_oc(std::string& point_cloud_path,std::string& compressed_path) {
	// Read point cloud 输入点云文件
	pcl::PointCloud<pcl::PointXYZRGB> sourceCloud;
	pcl::PLYReader reader;
	if (pcl::io::loadPLYFile(point_cloud_path, sourceCloud) == -1) {
		PCL_ERROR("Failed to load PLYFile!");
		return ;
	}

	bool showStatistics = true;
	pcl::io::compression_Profiles_e compressionProfile = pcl::io::MANUAL_CONFIGURATION;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
	PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compressionProfile, showStatistics, 0.001, 0.01,
		true, 100, true, 8);//输入参数
	std::stringstream compressedData;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());
	PointCloudEncoder->encodePointCloud(sourceCloud.makeShared(), compressedData);
	std::ofstream compressed_file(compressed_path, std::fstream::binary |std::fstream::out);
	compressed_file<< compressedData.str();
	compressed_file.close();

}

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
	std::cout << "Point cloud compression : " << std::endl;
	std::cout << "./point_cloud_compression point_cloud_path compressed_path "
		"cluster_method(eu, sv, km)"
		<< std::endl;
	std::string point_cloud_path = "C:/Users/Freeverc/Projects/point-cloud-processing/data/compression/longdress_vox10_1051.ply";
	std::string compressed_path = "C:/Users/Freeverc/Projects/point-cloud-processing/data/compression/compressed.bin";
	std::string uncompressed_path = "C:/Users/Freeverc/Projects/point-cloud-processing/data/compression/uncompressed.ply";
	std::string method = "oc";
	if (argc == 1) {
		std::cout << "No input file ! " << std::endl;
	}
	else if (argc == 2) {
		point_cloud_path = argv[1];
	}
	else if (argc == 3) {
		point_cloud_path = argv[1];
		compressed_path = argv[2];
	}
	else if (argc >= 4) {
		point_cloud_path = argv[1];
		compressed_path = argv[2];
		uncompressed_path = argv[3];
	}

	std::cout << "Input : " << point_cloud_path << std::endl;
	std::cout << "Compressed : " << compressed_path << std::endl;
	std::cout << "Uncompressed : " << uncompressed_path << std::endl;

	compress_by_oc(point_cloud_path, compressed_path);
	uncompress_by_oc(compressed_path, uncompressed_path);
	system("pause");
	return 0;
}
