#include "Mesh.h"

Mesh::Mesh() : num_vertices(0), num_faces(0) {}

Mesh::~Mesh() {}

bool Mesh::ReadFromObj(std::string file_name) {
  std::ifstream obj_file;
  obj_file.open(file_name);
  if (!obj_file.is_open())
    return false;
  std::string s;
  char c;
  while (getline(obj_file, s)) {
    std::stringstream ss(s);
    ss >> c;
    if (c == 'v') {
      float x, y, z;
      ss >> x >> y >> z;
      vertices.push_back(Point(x, y, z));
      ++num_vertices;
    } else if (c == 'f') {
      int id0, id1, id2;
      ss >> id0 >> id1 >> id2;
      faces.push_back(Face(id0, id1, id2));
      ++num_faces;
    }
  }
  return true;
}

bool Mesh::WriteToObj(std::string file_name) {
  // Writing polygon mesh to obj files.
  std::stringstream ss_obj;
  ss_obj << "####" << std::endl
         << "#" << std::endl
         << "####" << std::endl
         << "# OBJ File : " << file_name << std::endl
         << "#" << std::endl
         << "# Vertices: " << vertices.size() << std::endl
         << "# Faces: " << faces.size() << std::endl
         << "#" << std::endl
         << "####" << std::endl;

  // Write vertexes.
  for (size_t i = 0; i < vertices.size(); ++i) {
    ss_obj << "v " << vertices[i].x << " " << vertices[i].y << " "
           << vertices[i].z << std::endl;
  }

  // Write faces.
  for (size_t i = 0; i < faces.size(); ++i) {
    ss_obj << "f " << faces[i].idx0 << " " << faces[i].idx1 << " "
           << faces[i].idx2 << std::endl;
  }
  std::ofstream fs_obj;
  fs_obj.open(file_name, std::ios::out);
  fs_obj << ss_obj.str();
  fs_obj.close();
  std::cout << "Exported mesh to off file." << std::endl;
}

bool Mesh::WriteToPly(std::string file_name) {
  std::fstream text_file(file_name, std::ios::out);

  text_file << "ply" << std::endl;
  text_file << "format binary_little_endian 1.0" << std::endl;
  text_file << "element vertex " << vertices.size() << std::endl;
  text_file << "property float x" << std::endl;
  text_file << "property float y" << std::endl;
  text_file << "property float z" << std::endl;
  text_file << "element face " << faces.size() << std::endl;
  text_file << "property list uchar int vertex_index" << std::endl;
  text_file << "end_header" << std::endl;
  text_file.close();

  std::fstream binary_file(file_name,
                           std::ios::out | std::ios::binary | std::ios::app);

  for (const auto &vertex : vertices) {
    binary_file.write((char *)(&vertex.x), sizeof(float));
    binary_file.write((char *)(&vertex.y), sizeof(float));
    binary_file.write((char *)(&vertex.z), sizeof(float));
  }

  for (const auto &face : faces) {
    uint8_t a = 3;
    int id0 = face.idx0 - 1;
    int id1 = face.idx1 - 1;
    int id2 = face.idx2 - 1;
    binary_file.write((char *)(&a), sizeof(uint8_t));
    binary_file.write((char *)(&id0), sizeof(int));
    binary_file.write((char *)(&id1), sizeof(int));
    binary_file.write((char *)(&id2), sizeof(int));
  }

  binary_file.close();
  return true;
}