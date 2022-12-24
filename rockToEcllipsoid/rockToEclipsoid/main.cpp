#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "float.h"

#include "ceres/ceres.h"
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/IO/OBJ.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/smooth_mesh.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt K;
typedef K::Point_3 Point;
typedef K::Triangle_3 Face;
typedef CGAL::Surface_mesh<Point> Mesh;

namespace PMP = CGAL::Polygon_mesh_processing;

struct ExponentialResidual
{
  ExponentialResidual(double x, double y, double z)
      : x_(x), y_(y), z_(z) {}

  template <typename T>
  bool operator()(const T *const a, const T *const b, const T *const c, T *residual) const
  {
    residual[0] = 1.0 - x_ * x_ / *a - y_ * y_ / *b - z_ * z_ / *c;
    return true;
  }

private:
  // Observations for a sample.
  const double x_;
  const double y_;
  const double z_;
};

void GetEllipsolid(std::vector<Point> &points)
{
  double x_max = -DBL_MAX;
  double y_max = -DBL_MAX;
  double z_max = -DBL_MAX;
  double x_min = DBL_MAX;
  double y_min = DBL_MAX;
  double z_min = DBL_MAX;
  double x_mean = 0;
  double y_mean = 0;
  double z_mean = 0;
  for (int i = 0; i < points.size(); ++i)
  {
    double x = points[i].x().doubleValue();
    double y = points[i].y().doubleValue();
    double z = points[i].z().doubleValue();
    x_mean += x;
    y_mean += y;
    z_mean += z;
    x_max = std::max(x_max, x);
    y_max = std::max(y_max, y);
    z_max = std::max(z_max, z);
    x_min = std::min(x_min, x);
    y_min = std::min(y_min, y);
    z_min = std::min(z_min, z);
  }

  x_mean /= points.size();
  y_mean /= points.size();
  z_mean /= points.size();

  double a = (x_max - x_min) * (x_max - x_min) / 4;
  double b = (y_max - y_min) * (y_max - y_min) / 4;
  double c = (z_max - z_min) * (z_max - z_min) / 4;
  std::cout << "Mean : " << x_mean << " " << y_mean << " " << z_mean << std::endl;
  std::cout << "Min : " << x_min << " " << y_min << " " << z_min << std::endl;
  std::cout << "Max : " << x_max << " " << y_max << " " << z_max << std::endl;
  std::cout << "Initial : " << a << " " << b << " " << c << "\n";
  std::cout << "Initial sqrt: " << std::sqrt(a) << " " << std::sqrt(b) << " " << std::sqrt(c) << "\n";

  ceres::Problem problem;
  for (int i = 0; i < points.size(); ++i)
  {
    double x = points[i].x().doubleValue();
    double y = points[i].y().doubleValue();
    double z = points[i].z().doubleValue();
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 1, 1, 1>(
            new ExponentialResidual(x, y, z));
    problem.AddResidualBlock(cost_function, nullptr, &a, &b, &c);
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 500;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;

  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Final : " << a << " " << b << " " << c << "\n";
  std::cout << "Final sqrt: " << std::sqrt(a) << " " << std::sqrt(b) << " " << std::sqrt(c) << "\n";
}

int main(int argc, char *argv[])
{

  // Load OBJ
  Mesh mesh;
  std::vector<Point> points_ref;
  std::vector<std::vector<std::size_t>> faces_ref;
  std::string file(argv[1]);
  if (!CGAL::IO::read_OBJ(file, points_ref, faces_ref))
  {
    std::cout << "Failed reading" << std::endl;
    return 1;
  }

  double x_max = -DBL_MAX;
  double y_max = -DBL_MAX;
  double z_max = -DBL_MAX;
  double x_min = DBL_MAX;
  double y_min = DBL_MAX;
  double z_min = DBL_MAX;
  double x_mean = 0;
  double y_mean = 0;
  double z_mean = 0;
  for (int i = 0; i < points_ref.size(); ++i)
  {
    double x = points_ref[i].x().doubleValue();
    double y = points_ref[i].y().doubleValue();
    double z = points_ref[i].z().doubleValue();
    x_mean += x;
    y_mean += y;
    z_mean += z;
    x_max = std::max(x_max, x);
    y_max = std::max(y_max, y);
    z_max = std::max(z_max, z);
    x_min = std::min(x_min, x);
    y_min = std::min(y_min, y);
    z_min = std::min(z_min, z);
    mesh.add_vertex(points_ref[i]);
  }

  x_mean /= points_ref.size();
  y_mean /= points_ref.size();
  z_mean /= points_ref.size();
  std::cout << "Mean : " << x_mean << " " << y_mean << " " << z_mean << std::endl;
  std::cout << "Min : " << x_min << " " << y_min << " " << z_min << std::endl;
  std::cout << "Max : " << x_max << " " << y_max << " " << z_max << std::endl;

  std::vector<Point> points_new;
  for (int i = 0; i < points_ref.size(); ++i)
  {
    double x = points_ref[i].x().doubleValue() - x_mean;
    double y = points_ref[i].y().doubleValue() - y_mean;
    double z = points_ref[i].z().doubleValue() - z_mean;
    points_new.emplace_back(Point(x, y, z));
  }

  GetEllipsolid(points_new);

  return 0;
}