#include "Mesh.h"
#include "Geometry.h"
#include "omp.h"

Mesh::Mesh()
    : min_x_(FLT_MAX), min_y_(FLT_MAX), min_z_(FLT_MAX), max_x_(FLT_MIN),
    max_y_(FLT_MIN), max_z_(FLT_MIN) {}

Mesh::~Mesh() {}

bool Mesh::ReadFromObj(std::string file_name) {
    std::string mtl_file_name1(file_name);
    mtl_file_name1.replace(mtl_file_name1.size() - 3, 3, "mtl");
    std::ifstream mtl_file;
    mtl_file.open(mtl_file_name1);

    if (mtl_file.is_open()) {
        std::cout << "Reading mtl: " << mtl_file_name1 << std::endl;
    }
    else {
        std::string mtl_file_name2(file_name);
        mtl_file_name2 += ".mtl";
        mtl_file.open(mtl_file_name2);
        if (mtl_file.is_open()) {
            std::cout << "Reading mtl: " << mtl_file_name2 << std::endl;
        }
        else {
            std::cout << "No mtl file." << std::endl;
        }
    }


    if (mtl_file.is_open()) {
        std::string s;
        std::string m_name;
        while (std::getline(mtl_file, s)) {
            s = Trim(s);
            if (s.size() <= 1)
                continue;
            if (s[0] == '#')
                continue;
            std::vector<std::string> s_list;
            Split(s, s_list, " ");
            std::cout << s << std::endl;
            if (s_list[0] == "newmtl") {
                m_name = s_list[1];
                Material m(m_name);
                material_map[m_name] = m;
            }
            else if (s_list[0] == "Ns") {
                material_map[m_name].Ns = atof(s_list[1].c_str());
            }
            else if (s_list[0] == "Ni") {
                material_map[m_name].Ni = atof(s_list[1].c_str());
            }
            else if (s_list[0] == "d") {
                material_map[m_name].d = atof(s_list[1].c_str());
            }
            else if (s_list[0] == "Tr") {
                material_map[m_name].Tr = atof(s_list[1].c_str());
            }
            else if (s_list[0] == "Tf") {
                material_map[m_name].Tf.r = atof(s_list[1].c_str());
                material_map[m_name].Tf.g = atof(s_list[2].c_str());
                material_map[m_name].Tf.b = atof(s_list[3].c_str());
            }
            else if (s_list[0] == "illum") {
                material_map[m_name].illum = atof(s_list[1].c_str());
            }
            else if (s_list[0] == "Ka") {
                material_map[m_name].Ka.r = atof(s_list[1].c_str());
                material_map[m_name].Ka.g = atof(s_list[2].c_str());
                material_map[m_name].Ka.b = atof(s_list[3].c_str());
            }
            else if (s_list[0] == "Kd") {
                material_map[m_name].Kd.r = atof(s_list[1].c_str());
                material_map[m_name].Kd.g = atof(s_list[2].c_str());
                material_map[m_name].Kd.b = atof(s_list[3].c_str());
            }
            else if (s_list[0] == "Ks") {
                material_map[m_name].Ks.r = atof(s_list[1].c_str());
                material_map[m_name].Ks.g = atof(s_list[2].c_str());
                material_map[m_name].Ks.b = atof(s_list[3].c_str());
            }
            else if (s_list[0] == "Ke") {
                material_map[m_name].Ke.r = atof(s_list[1].c_str());
                material_map[m_name].Ke.g = atof(s_list[2].c_str());
                material_map[m_name].Ke.b = atof(s_list[3].c_str());
            }
        }
    }

    std::ifstream obj_file;
    obj_file.open(file_name);
    if (obj_file.is_open()) {
        std::cout << "Reading obj : " << file_name << std::endl;
        std::string s;
        int s_id = 0;
        std::string o_name = "";
        std::string g_name = "";
        std::string mtl_name = "";

        while (std::getline(obj_file, s)) {
            s = Trim(s);
            if (s.size() <= 1)
                continue;
            if (s[0] == '#')
                continue;
            std::vector<std::string> s_list;
            Split(s, s_list, " ");
            if (s_list[0] == "v") {
                double x = atof(s_list[1].c_str());
                double y = atof(s_list[2].c_str());
                double z = atof(s_list[3].c_str());

                //std::cout << "v : " << x << " " << y << " " <<z << std::endl;
                if (x < min_x_)
                    min_x_ = x;
                if (y < min_y_)
                    min_y_ = y;
                if (z < min_z_)
                    min_z_ = z;
                if (x > max_x_)
                    max_x_ = x;
                if (y > max_y_)
                    max_y_ = y;
                if (z > max_z_)
                    max_z_ = z;
                vertice_list.push_back(Point(x, y, z));
            }
            else if (s_list[0] == "vn") {
                double x = atof(s_list[1].c_str());
                double y = atof(s_list[2].c_str());
                double z = atof(s_list[3].c_str());
                vertice_normal_list.push_back(Point(x, y, z));
            }
            else if (s_list[0] == "s") {
                s_id = atoi(s_list[1].c_str());
                smooth_group_map[s_id] += 1;
            }
            else if (s_list[0] == "o") {
                object_name = s_list[1];
            }
            else if (s_list[0] == "g") {
                group_name = s_list[1];
            }
            else if (s_list[0] == "usemtl") {
                mtl_name = s_list[1];
            }
            else if (s_list[0] == "f") {
                Face f;
                for (int j = 1; j < s_list.size(); ++j) {
                    std::vector<std::string> v_list;
                    Split(s_list[j], v_list, "/");
                    //std::cout << "f " << j << std::endl;
                    //for (int k = 0; k < v_list.size(); ++k) {
                       // std::cout << v_list[k] << std::endl;
                    //}
                    if (v_list.size() == 3) {
                        f.v_list.push_back(atoi(v_list[0].c_str()));
                        f.vt_list.push_back(atoi(v_list[0].c_str()));
                        f.vn_list.push_back(atoi(v_list[0].c_str()));
                    }
                    else if (v_list.size() == 2) {
                        f.v_list.push_back(atoi(v_list[0].c_str()));
                        f.vn_list.push_back(atoi(v_list[0].c_str()));
                    }
                    else if (v_list.size() == 1) {
                        f.v_list.push_back(atoi(v_list[0].c_str()));
                    }
                }
                f.smooth_group_id = s_id;
                f.material_name = mtl_name;
                face_list.push_back(f);
            }
        }

        std::cout << "o name: " << object_name << std::endl;
        std::cout << "g name: " << group_name << std::endl;

        std::cout << "v: " << vertice_list.size() << std::endl;
        std::cout << "vn: " << vertice_normal_list.size() << std::endl;
        std::cout << "f: " << face_list.size() << std::endl;
    }

    return true;
}


void Split(const std::string& s, std::vector<std::string>& tokens,
    const std::string& delimiters) {
    std::string::size_type lastPos = s.find_first_not_of(delimiters, 0);
    std::string::size_type pos = s.find_first_of(delimiters, lastPos);
    while (std::string::npos != pos || std::string::npos != lastPos) {
        tokens.push_back(
            s.substr(lastPos, pos - lastPos)); // use emplace_back after C++11
        lastPos = s.find_first_not_of(delimiters, pos);
        pos = s.find_first_of(delimiters, lastPos);
    }
}
std::string Trim(std::string& str) {
    str.erase(0, str.find_first_not_of(" \t"));
    str.erase(str.find_last_not_of(" \t") + 1);
    return str;
}

void SliceScan(Mesh& mesh, std::vector<cv::Mat*>& images, double r_grid, double r_slice, int thickness) {
    int size_y = std::ceil((mesh.max_y_ - mesh.min_y_) / r_slice);
    int size_x = std::ceil((mesh.max_x_ - mesh.min_x_) / r_grid);
    int size_z = std::ceil((mesh.max_z_ - mesh.min_z_) / r_grid);
    std::cout << "Size : " << size_x << " " << size_y << " " << size_z
        << std::endl;

    for (int i = 0; i < mesh.face_list.size(); ++i) {
        if (mesh.face_list[i].v_list.size() < 3) {
            for (int j = 0; j < mesh.face_list[i].v_list.size(); ++j) {
                std::cout << mesh.face_list[i].v_list[j] << " ";
            }
            std::cout << std::endl;
            continue;
        }
        int v0 = mesh.face_list[i].v_list[0] - 1;
        int v1 = mesh.face_list[i].v_list[1] - 1;
        int v2 = mesh.face_list[i].v_list[2] - 1;
        Point p0 = mesh.vertice_list[v0];
        Point p1 = mesh.vertice_list[v1];
        Point p2 = mesh.vertice_list[v2];
        std::string mtl_name = mesh.face_list[i].material_name;
        int r = 255 * mesh.material_map[mtl_name].Kd.r;
        int g = 255 * mesh.material_map[mtl_name].Kd.g;
        int b = 255 * mesh.material_map[mtl_name].Kd.b;
        cv::Scalar color(b, g, r);

        double min_x = std::min(std::min(p0.x, p1.x), p2.x);
        double max_x = std::max(std::max(p0.x, p1.x), p2.x);
        double min_z = std::min(std::min(p0.z, p1.z), p2.z);
        double max_z = std::max(std::max(p0.z, p1.z), p2.z);
        double min_y = std::min(std::min(p0.y, p1.y), p2.y);
        double max_y = std::max(std::max(p0.y, p1.y), p2.y);

        //std::cout << "v: " << v0 << " " << v1 << " " << v2 << std::endl;
        //std::cout << "min: " << min_x << " " << min_y << " " <<min_z << std::endl;
        //std::cout << "max: " << max_x << " " << max_y << " " << max_z << std::endl;
        //std::cout << "p0: " << p0.x << " " << p0.y << " " << p0.z << std::endl;
        //std::cout << "p1: " << p1.x << " " << p1.y << " " << p1.z << std::endl;
        //std::cout << "p2: " << p2.x << " " << p2.y << " " << p2.z << std::endl;

        for (double y = min_y; y <= max_y; y += r_slice) {
            int y_i = (y - mesh.min_y_) / r_slice;
            if (std::abs(p0.y - p1.y) < 1e-6 && std::abs(p0.y - p2.y) < 1e-6) {
                cv::Point root_points[1][3];
                root_points[0][0] = cv::Point((p0.z - mesh.min_z_) / r_grid,
                    (p0.x - mesh.min_x_) / r_grid);
                root_points[0][1] = cv::Point((p1.z - mesh.min_z_) / r_grid,
                    (p1.x - mesh.min_x_) / r_grid);
                root_points[0][2] = cv::Point((p2.z - mesh.min_z_) / r_grid,
                    (p2.x - mesh.min_x_) / r_grid);
                const cv::Point* pts[1] = { root_points[0] };
                int npts[] = { 3 };

                if (!images[y_i])
                    images[y_i] =
                    new cv::Mat(size_x, size_z, CV_8UC3, cv::Scalar(255, 255, 255));

                cv::fillPoly(*images[y_i], pts, npts, 1, color);

            }
            else {
                double k_0_1 =
                    std::abs(p1.y - p0.y) < 1e-6 ? -1 : (y - p0.y) / (p1.y - p0.y);
                double k_0_2 =
                    std::abs(p2.y - p0.y) < 1e-6 ? -1 : (y - p0.y) / (p2.y - p0.y);
                double k_1_2 =
                    std::abs(p2.y - p1.y) < 1e-6 ? -1 : (y - p1.y) / (p2.y - p1.y);

                double x1, z1, x2, z2;
                if (k_0_1 < 0 || k_0_1 > 1) {
                    if ((k_0_2 < 0 || k_0_2 > 1) || (k_1_2 < 0 || k_1_2 > 1)) {
                        std::cout << 1 << std::endl;
                        std::cout << k_0_1 << " " << k_0_2 << " " << k_1_2 << std::endl;
                        std::cout << y << " " << p0.y << " " << p1.y << " " << p2.y << std::endl;
                        continue;
                    }
                    k_0_2 = std::min(k_0_2, 1.0);
                    k_1_2 = std::min(k_1_2, 1.0);
                    k_0_2 = std::max(k_0_2, 0.0);
                    k_1_2 = std::max(k_1_2, 0.0);
                    x1 = p0.x + k_0_2 * (p2.x - p0.x);
                    z1 = p0.z + k_0_2 * (p2.z - p0.z);
                    x2 = p1.x + k_1_2 * (p2.x - p1.x);
                    z2 = p1.z + k_1_2 * (p2.z - p1.z);
                }
                else if (k_0_2 < 0 || k_0_2 > 1) {
                    if ((k_0_1 < 0 || k_0_1 > 1) || (k_1_2 < 0 || k_1_2 > 1)) {
                        std::cout << 2 << std::endl;
                        std::cout << k_0_1 << " " << k_0_2 << " " << k_1_2 << std::endl;
                        std::cout << y << " " << p0.y << " " << p1.y << " " << p2.y << std::endl;
                        continue;
                    }
                    k_0_1 = std::min(k_0_1, 1.0);
                    k_1_2 = std::min(k_1_2, 1.0);
                    k_0_1 = std::max(k_0_1, 0.0);
                    k_1_2 = std::max(k_1_2, 0.0);
                    x1 = p0.x + k_0_1 * (p1.x - p0.x);
                    z1 = p0.z + k_0_1 * (p1.z - p0.z);
                    x2 = p1.x + k_1_2 * (p2.x - p1.x);
                    z2 = p1.z + k_1_2 * (p2.z - p1.z);
                }
                else if (k_1_2 < 0 || k_1_2 > 1) {
                    if ((k_0_2 < 0 || k_0_2 > 1) || (k_0_1 < 0 || k_0_1 > 1)) {
                        std::cout << 3 << std::endl;
                        std::cout << k_0_1 << " " << k_0_2 << " " << k_1_2 << std::endl;
                        std::cout << y << " " << p0.y << " " << p1.y << " " << p2.y << std::endl;
                        continue;
                    }
                    k_0_1 = std::min(k_0_1, 1.0);
                    k_0_2 = std::min(k_0_2, 1.0);
                    k_0_1 = std::max(k_0_1, 0.0);
                    k_0_2 = std::max(k_0_2, 0.0);
                    x1 = p0.x + k_0_1 * (p1.x - p0.x);
                    z1 = p0.z + k_0_1 * (p1.z - p0.z);
                    x2 = p0.x + k_0_2 * (p2.x - p0.x);
                    z2 = p0.z + k_0_2 * (p2.z - p0.z);
                }
                else {
                    std::cout << 4 << std::endl;
                    std::cout << k_0_1 << " " << k_0_2 << " " << k_1_2 << std::endl;
                    std::cout << y << " " << p0.y << " " << p1.y << " " << p2.y << std::endl;
                    x1 = p0.x + k_0_1 * (p1.x - p0.x);
                    z1 = p0.z + k_0_1 * (p1.z - p0.z);
                    x2 = p0.x + k_0_2 * (p2.x - p0.x);
                    z2 = p0.z + k_0_2 * (p2.z - p0.z);
                    if (std::abs(x1 - x2) < 1e-6) {
                        x2 = p1.x + k_1_2 * (p2.x - p1.x);
                        z2 = p1.z + k_1_2 * (p2.z - p1.z);
                    }
                }

                if (x1 < min_x || x1 > max_x || x2 < min_x || x2 > max_x) {
                    std::cout << "k: " << k_0_1 << " " << k_0_2 << " " << k_1_2
                        << std::endl;
                    std::cout << "range x " << min_x << " " << max_x << " x: " << x1
                        << " " << x2 << std::endl;
                }
                if (z1 < min_z || z1 > max_z || z2 < min_z || z2 > max_z) {
                    std::cout << "k: " << k_0_1 << " " << k_0_2 << " " << k_1_2
                        << std::endl;
                    std::cout << "range z " << min_z << " " << max_z << " z: " << z1
                        << " " << z2 << std::endl;
                }

                int x1_i = (x1 - mesh.min_x_) / r_grid;
                int z1_i = (z1 - mesh.min_z_) / r_grid;
                int x2_i = (x2 - mesh.min_x_) / r_grid;
                int z2_i = (z2 - mesh.min_z_) / r_grid;
                if (!images[y_i])
                    images[y_i] =
                    new cv::Mat(size_x, size_z, CV_8UC3, cv::Scalar(255, 255, 255));
                cv::line(*images[y_i], cv::Point(z1_i, x1_i), cv::Point(z2_i, x2_i),
                    color, thickness);
            }
        }
        if (i % 10000 == 0)
            std::cout << "Processed " << i << " faces. " << std::endl;
    }
}

void SliceFilling(Mesh& mesh, std::vector<cv::Mat*>& images, double r_grid, double r_slice, int thickness) {
    int size_y = std::ceil((mesh.max_y_ - mesh.min_y_) / r_slice);
    int size_x = std::ceil((mesh.max_x_ - mesh.min_x_) / r_grid);
    int size_z = std::ceil((mesh.max_z_ - mesh.min_z_) / r_grid);
    std::cout << "Size : " << size_x << " " << size_y << " " << size_z
        << std::endl;

	omp_set_num_threads(4);

    for (int y_i = 0; y_i < size_y; ++y_i)
    {
        double y = mesh.min_y_ + y_i * r_slice + 0.5 * r_slice;
        std::cout << "Processing layer " << y_i << " " << y << " " << size_y << std::endl;
        if (!images[y_i])
        {
            images[y_i] =
                new cv::Mat(size_x, size_z, CV_8UC3, cv::Scalar(255, 255, 255));
        }

#pragma omp parallel for  num_threads(4) 
        for (int x_i = 0; x_i < size_x; ++x_i)
        {
            double x = mesh.min_x_ + x_i * r_grid + 0.5 * r_grid;

            for (int z_i = 0; z_i < size_z; ++z_i)
            {
                double z = mesh.min_z_ + z_i * r_grid + 0.5 * r_grid;
                Vector3d  p(x, y, z);
                int r = 0;
                int g = 0;
                int b = 0;
                double d = INT_MAX;

                int cross_num_x = 0;
                int cross_num_y = 0;
                int cross_num_z = 0;

                for (int i = 0; i < mesh.face_list.size(); ++i)
                {
                    int p0_i = mesh.face_list[i].v_list[0] - 1;
                    int p1_i = mesh.face_list[i].v_list[1] - 1;
                    int p2_i = mesh.face_list[i].v_list[2] - 1;
                    Point p0 = mesh.vertice_list[p0_i];
                    Point p1 = mesh.vertice_list[p1_i];
                    Point p2 = mesh.vertice_list[p2_i];

                    Vector3d  v0(p0.x, p0.y, p0.z);
                    Vector3d  v1(p1.x, p1.y, p1.z);
                    Vector3d  v2(p2.x, p2.y, p2.z);

                    Vector3d  d_x(1, 0, 0);
                    Vector3d  d_y(0, 1, 0);
                    Vector3d  d_z(0, 0, 1);

                    Vector3d  I(0, 0, 0);
                    if (ray_triangle_intersection(v0, v1, v2, p, d_x, &I))
                    {
                        ++cross_num_x;
                    }
                    if (ray_triangle_intersection(v0, v1, v2, p, d_y, &I))
                    {
                        ++cross_num_y;
                    }
                    if (ray_triangle_intersection(v0, v1, v2, p, d_z, &I))
                    {
                        ++cross_num_z;
                    }

                    Vector3d  v_m((p0.x + p1.x + p2.x) / 3, (p0.y + p1.y + p2.y) / 3, (p0.z + p1.z + p2.z) / 3);
                    double d_new = std::min(std::min(p.Distace2(v0), p.Distace2(v1)), std::min(p.Distace2(v2), p.Distace2(v_m)));
                    if (d_new < d)
                    {
                        d = d_new;
                        std::string mtl_name = mesh.face_list[i].material_name;
                        r = 255 * mesh.material_map[mtl_name].Kd.r;
                        g = 255 * mesh.material_map[mtl_name].Kd.g;
                        b = 255 * mesh.material_map[mtl_name].Kd.b;
                    }

                }

                int cross_num = (cross_num_x % 2) + (cross_num_y % 2) + (cross_num_z % 2);
                if (cross_num>= 2)
                {
                    images[y_i]->at<cv::Vec3b>(x_i, z_i)[0] = b;
                    images[y_i]->at<cv::Vec3b>(x_i, z_i)[1] = g;
                    images[y_i]->at<cv::Vec3b>(x_i, z_i)[2] = r;
                }
            }
        }
        std::cout << "Finished layer " << y_i << std::endl;
    }
}
