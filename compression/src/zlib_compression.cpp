#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "zpipe.h"

void compress_by_zip(std::string& pointcloud_path, std::string& compressed_path) {
	Zpipe zpipe;
	// Read point cloud 输入点云文件
	FILE* unzipped = fopen(pointcloud_path.c_str(), "rb");
	FILE* zipped  = fopen(compressed_path.c_str(),  "w");
	SET_BINARY_MODE(unzipped);
	SET_BINARY_MODE(zipped);

	int  ret = zpipe.def(unzipped, zipped, Z_DEFAULT_COMPRESSION);
	if (ret != Z_OK) {
		zpipe.zerr(ret);
	}
}


int main(int argc, char* argv[])
{    
	std::cout << "Point cloud compression : " << std::endl;
	std::cout << "./point_cloud_compression point_cloud_path compressed_path "
		"cluster_method(eu, sv, km)"
		<< std::endl;
	std::string point_cloud_path = "C:/Users/Freeverc/Projects/point-cloud-processing/data/compression/longdress_vox10_1051.ply";
	std::string compressed_path = "C:/Users/Freeverc/Projects/point-cloud-processing/data/compression/compressed_zlib.bin";
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
	else {
		std::cout << "Too many input file ! " << std::endl;
	}

	std::cout << "Input : " << point_cloud_path << std::endl;

	compress_by_zip(point_cloud_path, compressed_path);
	std::cout << "Compressed : " << compressed_path << std::endl;
	return 0;
}

