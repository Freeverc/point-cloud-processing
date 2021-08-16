#include "Mesh.h"
#include <iostream>

int main(int argc, char *argv[]) {
  std::string file_name;
  if (argc == 1) {
    std::cout << "No input file ! " << std::endl;
  } else if (argc > 2) {
    std::cout << "Too many input arguments." << std::endl;
    return 0;
  }
  file_name = argv[1];
  int pos = file_name.find("obj");
  if (pos < 0) {
    std::cout << "Not an obj file ! " << std::endl;
    return false;
  }

  Mesh mesh;
  mesh.ReadFromObj(file_name);
  std::cout << mesh.num_vertices << " " << mesh.num_faces << std::endl;
  std::string output_file_name(file_name);
  output_file_name.replace(pos, 3, "ply");

  std::cout << file_name << std::endl;
  std::cout << output_file_name << std::endl;
  mesh.WriteToPly(output_file_name);

  return 0;
}