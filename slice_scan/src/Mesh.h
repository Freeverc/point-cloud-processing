#pragma once

#include "float.h"
#include "limits.h"
#include "math.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// The struct for vertex, each vertex has x,y,z coordinates.
struct NormalColor {
    double r = 0;
    double g = 0;
    double b = 0;
};

struct Point {
    double x, y, z;
    uint8_t r, g, b;
    double xn, yn, zn;
    Point() {};
    Point(double _x, double _y, double _z) {
        x = _x;
        y = _y;
        z = _z;
    };
};

struct Material {
    double Ns;        // 材质的光亮度
    double Ni;        // 材质
    double d;         //可用于定义材质的Alpha透明度
    double Tr;        //可用于定义材质的Alpha透明度
    NormalColor Tf;   //可用于定义材质的Alpha透明度
    double illum;     //照明度（illumination）后可接0~10范围参数
    NormalColor Ka;   //材质的环境光（ambient color）
    NormalColor Kd;   //散射光（diffuse color）
    NormalColor Ks;   //镜面光（specular color）
    NormalColor Ke;   //放射光（emissive color）
    double sharpness; //材质的锐度（sharpness）
    std::string name; //名称

    Material() {};
    Material(std::string new_name) { name = new_name; };
    void Print() {
        std::cout << name << std::endl;
        std::cout << Ns << std::endl;
        std::cout << Ni << std::endl;
        std::cout << d << std::endl;
        std::cout << Tr << std::endl;
        std::cout << Tf.r << " " << Tf.g << " " << Tf.b << std::endl;
        std::cout << illum << std::endl;
        std::cout << Ka.r << " " << Ka.g << " " << Ka.b << std::endl;
        std::cout << Kd.r << " " << Kd.g << " " << Kd.b << std::endl;
        std::cout << Ks.r << " " << Ks.g << " " << Ks.b << std::endl;
        std::cout << Ke.r << " " << Ke.g << " " << Ke.b << std::endl;
    }
};

// The struct for face, each face is a triangle and has three vertex indexs.
struct Face {
    std::vector<int> v_list;
    std::vector<int> vt_list;
    std::vector<int> vn_list;
    int smooth_group_id;
    std::string material_name;
    Face() {};
};

// The class for Mesh.
class Mesh {
public:
    Mesh();
    ~Mesh();

    // Read mesh from OBJ file.
    bool ReadFromObj(std::string file_name);

    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
    double min_z_;
    double max_z_;

    std::string object_name;
    std::string group_name;
    std::unordered_map<int, int> smooth_group_map;

    std::vector<Point> vertice_list;
    std::vector<Point> vertice_normal_list;
    std::vector<Face> face_list;
    std::unordered_map<std::string, Material> material_map;
};

void Split(const std::string& s, std::vector<std::string>& tokens,
    const std::string& delimiters = " ");

std::string Trim(std::string& str);

void SliceScan(Mesh& mesh, std::vector<cv::Mat*>& images, double r_grid = 0.5, double r_slice = 0.5,
    int thickness = 3);

void SliceFilling(Mesh& mesh, std::vector<cv::Mat*>& images, double r_grid = 0.5, double r_slice = 0.5,
    int thickness = 3);
