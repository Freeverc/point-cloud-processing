#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "zpipe.h"

void uncompress_by_zip(std::string& compressed_path, std::string& uncompressed_path) {
	Zpipe zpipe;
	// Read point cloud 输入点云文件
	FILE* zipped = fopen(compressed_path.c_str(), "rb");
	FILE* unzipped = fopen(uncompressed_path.c_str(), "w");
	SET_BINARY_MODE(zipped);
	SET_BINARY_MODE(unzipped);
	int  ret = zpipe.inf(zipped, unzipped);
	if (ret != Z_OK) {
		zpipe.zerr(ret);
	}
}


int main(int argc, char* argv[])
{
	SET_BINARY_MODE(stdin);
	SET_BINARY_MODE(stdout);
	std::cout << "Point cloud uncompression by zip : " << std::endl;
	std::cout << "./zip_uncompression point_cloud_path compressed_path "
		<< std::endl;
	std::string compressed_path = "C:/Users/Freeverc/Projects/point-cloud-processing/data/compression/compressed_zlib.bin";
	std::string uncompressed_path = "C:/Users/Freeverc/Projects/point-cloud-processing/data/compression/uncompressed_zlib.ply";
	std::string method = "oc";
	if (argc == 1) {
		std::cout << "No input file ! " << std::endl;
	} else if (argc == 2) {
		compressed_path= argv[1];
	}
	else if (argc == 3) {
		compressed_path= argv[1];
		uncompressed_path = argv[2];
	}
	else  {
		std::cout << "Too many input file ! " << std::endl;
	}

	std::cout << "Input : " << compressed_path << std::endl;
	uncompress_by_zip(compressed_path, uncompressed_path);
	std::cout << "Uncompressed : " << uncompressed_path << std::endl;
	return 0;
}

