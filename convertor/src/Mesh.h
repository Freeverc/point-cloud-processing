#ifndef Mesh_H
#define Mesh_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// The struct for vertex, each vertex has x,y,z coordinates.
struct Point {
  float x, y, z;
  Point(float _x, float _y, float _z) {
    x = _x;
    y = _y;
    z = _z;
  };
};

// The struct for face, each face is a triangle and has three vertex indexs.
struct Face {
  int idx0, idx1, idx2;
  Face(int _idx0, int _idx1, int _idx2) {
    idx0 = _idx0;
    idx1 = _idx1;
    idx2 = _idx2;
  };
};

// The class for Mesh.
class Mesh {
public:
  Mesh();
  ~Mesh();

  // Read mesh from OBJ file.
  bool ReadFromObj(std::string file_name);

  // Read mesh from OBJ file.
  bool ReadFromPly(std::string file_name);

  // Write mesh to OBJ file.
  bool WriteToObj(std::string file_name);

  // Write mesh to OBJ file.
  bool WriteToPly(std::string file_name);

  int num_vertices;
  int num_faces;

private:
  std::vector<Point> vertices;
  std::vector<Face> faces;
};

#endif