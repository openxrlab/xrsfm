//
// Created by SENSETIME\yezhichao1 on 2020/12/30.
//

#include "triangluate_svd.h"

double CalculateTriangulationAngle(const Eigen::Vector3d &center1, const Eigen::Vector3d &center2,
                                   const Eigen::Vector3d &point3D) {
  const double baseline_length_squared = (center1 - center2).squaredNorm();

  const double ray_length_squared1 = (point3D - center1).squaredNorm();
  const double ray_length_squared2 = (point3D - center2).squaredNorm();

  // Using "law of cosines" to compute the enclosing angle between rays.
  const double denominator = 2.0 * std::sqrt(ray_length_squared1 * ray_length_squared2);
  if (denominator == 0.0) {
    return 0.0;
  }
  const double nominator = ray_length_squared1 + ray_length_squared2 - baseline_length_squared;
  const double angle = std::abs(std::acos(nominator / denominator));

  // Triangulation is unstable for acute angles (far away points) and
  // obtuse angles (close points), so always compute the minimum angle
  // between the two intersecting rays.
  return std::min(angle, M_PI - angle);
}

itslam::vector<4> triangulate_point(const std::vector<itslam::matrix<3, 4>> &Ps,
                                    const std::vector<itslam::vector<2>> &points) {
  itslam::matrix<Eigen::Dynamic, 4> A(points.size() * 2, 4);
  for (size_t i = 0; i < points.size(); ++i) {
    A.row(i * 2 + 0) = points[i](0) * Ps[i].row(2) - Ps[i].row(0);
    A.row(i * 2 + 1) = points[i](1) * Ps[i].row(2) - Ps[i].row(1);
  }
  return A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3);
}

bool triangulate_point_scored(const std::vector<itslam::matrix<3, 4>> &Ps, const std::vector<itslam::vector<2>> &points,
                              itslam::vector<3> &p, double &score) {
  if (Ps.size() < 2) return false;
  itslam::vector<4> q = triangulate_point(Ps, points);
  score = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    itslam::vector<3> qi = Ps[i] * q;
    if (!(qi[2] * q[3] > 0)) {  // z<0
      return false;
    }
    if (!(qi[2] / q[3] < 100)) {  // z>100
      return false;
    }
    score += (qi.hnormalized() - points[i]).squaredNorm();
    //        std::cout << (qi.hnormalized() - points[i]).norm() * 700 << " ";
  }
  //    std::cout << std::endl;
  score /= points.size();
  p = q.hnormalized();
  return true;
}

bool triangulate_point_checked(const std::vector<itslam::matrix<3, 4>> &Ps,
                               const std::vector<itslam::vector<2>> &points, itslam::vector<3> &p) {
  double score;
  return triangulate_point_scored(Ps, points, p, score);
}
