#include "Mesh.h"
#include <iostream>

int main(int argc, char *argv[]) {
  std::string file_name = "air2.obj";
  float resolution_grid = 0.5;
  float resolution_layer = 0.5;
  if (argc == 1) {
    std::cout << "No input file ! " << std::endl;
  } else if (argc == 2) {
    file_name = argv[1];
    std::cout << "File name: " << file_name << std::endl;
  } else if (argc == 3) {
    file_name = argv[1];
    resolution_grid = atof(argv[2]);
    resolution_layer = atof(argv[2]);
    std::cout << "File name: " << file_name << std::endl;
    std::cout << "Resolution grid: " << resolution_grid << std::endl;
    std::cout << "Resolution layer: " << resolution_layer << std::endl;
  } else if (argc == 4) {
    file_name = argv[1];
    resolution_grid = atof(argv[2]);
    resolution_layer = atof(argv[3]);
    std::cout << "File name: " << file_name << std::endl;
    std::cout << "Resolution grid: " << resolution_grid << std::endl;
    std::cout << "Resolution layer: " << resolution_layer << std::endl;
  }
  file_name = argv[1];
  int pos = file_name.find("obj");
  if (pos < 0) {
    std::cout << "Not an obj file ! " << std::endl;
    return false;
  }

  Mesh mesh;
  mesh.ReadFromObj(file_name);
  std::cout << " v :" << mesh.vertice_list.size()
            << " n :" << mesh.vertice_normal_list.size()
            << " f: " << mesh.face_list.size() << std::endl;
  std::cout << "x range: " << mesh.min_x_ << " " << mesh.max_x_ << std::endl;
  std::cout << "y range: " << mesh.min_y_ << " " << mesh.max_y_ << std::endl;
  std::cout << "z range: " << mesh.min_z_ << " " << mesh.max_z_ << std::endl;

  std::cout << "Generating slice scanned images." << std::endl;

  SliceScan(mesh, resolution_grid, resolution_layer);


  return 0;
}