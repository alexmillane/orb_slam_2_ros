#include "orb_slam_2_ros/visualization.hpp"

#include <glog/logging.h>

#include <Eigen/Eigenvalues>

#include <eigen_conversions/eigen_msg.h>

namespace orb_slam_2_interface {
namespace visualization {

void addFrameToMarkerArray(const Transformation& T_M_C,
                           const std::string& frame,
                           const FrameType& frame_type,
                           visualization_msgs::MarkerArray* marker_array_ptr,
                           size_t* axes_marker_index_ptr) {
  // Argument checks
  CHECK_NOTNULL(marker_array_ptr);
  CHECK_NOTNULL(axes_marker_index_ptr);
  // Getting the markers for the axes
  std::vector<visualization_msgs::Marker> markers;
  std::string ns;
  switch (frame_type) {
    case FrameType::KeyFrame:
      drawAxesArrowsWithColor(T_M_C.getPosition(), T_M_C.getEigenQuaternion(),
                              kAxesScale, kAxesDiameter, kKeyframeColors,
                              kAxesAlpha, &markers);
      ns = "keyframe";
      break;
    case FrameType::Frame:
      drawAxesArrowsWithColor(T_M_C.getPosition(), T_M_C.getEigenQuaternion(),
                              kAxesScale, kAxesDiameter, kFrameColors,
                              kAxesAlpha, &markers);
      ns = "frame";
      break;
    case FrameType::OptimizedKeyFrame:
      drawAxesArrowsWithColor(T_M_C.getPosition(), T_M_C.getEigenQuaternion(),
                              kAxesScale, kAxesDiameter,
                              kOptimizedKeyframeColors, kAxesAlpha, &markers);
      ns = "optimized_keyframe";
      break;
    case FrameType::OptimizedPatchFrame:
      drawAxesArrowsWithColor(T_M_C.getPosition(), T_M_C.getEigenQuaternion(),
                              kPatchAxesScale, kPatchAxesDiameter,
                              kOptimizedPatchframeColors, kPatchAxesAlpha,
                              &markers);
      ns = "optimized_patchframe";
      break;
  }
  // Setting the marker frames
  size_t marker_counter = 0;
  for (auto marker : markers) {
    // Adding the header info to the marker
    marker.header.frame_id = frame;
    marker.ns = ns;
    marker.id = *axes_marker_index_ptr + marker_counter;
    // Pushing it onto the array
    marker_array_ptr->markers.push_back(marker);
    // Incrementing the counter
    marker_counter++;
  }
  *axes_marker_index_ptr += marker_counter;
}

// Helper function to create a std_msgs::ColorRGBA.
inline std_msgs::ColorRGBA createColorRGBA(float r, float g, float b, float a) {
  std_msgs::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

void drawArrowPoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                     const std_msgs::ColorRGBA& color, double diameter,
                     visualization_msgs::Marker* marker) {
  marker->type = visualization_msgs::Marker::ARROW;
  marker->action = visualization_msgs::Marker::ADD;
  marker->color = color;

  marker->points.resize(2);
  tf::pointEigenToMsg(p1, marker->points[0]);
  tf::pointEigenToMsg(p2, marker->points[1]);

  marker->scale.x = diameter * 0.1;
  marker->scale.y = diameter * 2 * 0.1;
  marker->scale.z = 0;
}

void drawAxesArrowsWithColor(const Eigen::Vector3d& p,
                             const Eigen::Quaterniond& q, double scale,
                             double diameter, const double colors[3][3],
                             double alpha,
                             std::vector<visualization_msgs::Marker>* markers) {
  markers->resize(3);
  Eigen::Vector3d origin;
  origin.setZero();
  // Creating the axis markers
  visualization_msgs::Marker unit_x;
  visualization_msgs::Marker unit_y;
  visualization_msgs::Marker unit_z;
  drawArrowPoints(
      origin + p, q * Eigen::Vector3d::UnitX() * scale + p,
      createColorRGBA(colors[0][0], colors[0][1], colors[0][2], alpha),
      diameter, &unit_x);
  drawArrowPoints(
      origin + p, q * Eigen::Vector3d::UnitY() * scale + p,
      createColorRGBA(colors[1][0], colors[1][1], colors[1][2], alpha),
      diameter, &unit_y);
  drawArrowPoints(
      origin + p, q * Eigen::Vector3d::UnitZ() * scale + p,
      createColorRGBA(colors[2][0], colors[2][1], colors[2][2], alpha),
      diameter, &unit_z);
  // Adding the markers to the array.
  markers->push_back(unit_x);
  markers->push_back(unit_y);
  markers->push_back(unit_z);
}

void addCovarianceEllipseToMarkerArray(
    const Transformation& T_M_C, const Eigen::Matrix3d& covariance_C,
    const std::string& frame, const CovarianceType& covariance_type,
    visualization_msgs::MarkerArray* marker_array_ptr,
    size_t* axes_marker_index_ptr) {
  // Argument checks
  CHECK_NOTNULL(marker_array_ptr);
  CHECK_NOTNULL(axes_marker_index_ptr);

  // Rotating the covariance into the world frame for display
  Eigen::Matrix3d covariance_W = T_M_C.getRotationMatrix() * covariance_C *
                                 T_M_C.getRotationMatrix().transpose();

  // Getting the markers for the axes
  visualization_msgs::Marker marker;
  std::string ns;
  switch (covariance_type) {
    case CovarianceType::OptimizedKeyFrame:
      drawCovariance3D(T_M_C.getPosition(), covariance_W,
                       kKeyframeCovarianceColor, kKeyframeCovarianceAlpha,
                       kKeyframeCovarianceScale, &marker);
      ns = "covariance_keyframe";
      break;
    case CovarianceType::OptimizedPatchFrame:
      drawCovariance3D(T_M_C.getPosition(), covariance_W, kPatchCovarianceColor,
                       kPatchCovarianceAlpha, kPatchCovarianceScale, &marker);
      ns = "covariance_patch";
  }
  // Filling out the other fields
  marker.header.frame_id = frame;
  marker.ns = ns;
  marker.id = *axes_marker_index_ptr;
  // Pushing onto the array
  marker_array_ptr->markers.push_back(marker);
  (*axes_marker_index_ptr)++;
}

void drawCovariance3D(const Eigen::Vector3d& mu, const Eigen::Matrix3d& cov,
                      const double colors[3], double alpha, double n_sigma,
                      visualization_msgs::Marker* marker) {
  // Adding easier color stuff
  const std_msgs::ColorRGBA color = createColorRGBA(colors[0], colors[1], colors[2], alpha);
  // This operation just guaruntees the matrix is self adjoint
  const Eigen::Matrix3d covariance_self_adjoint = (cov + cov.transpose()) * 0.5;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
      covariance_self_adjoint, Eigen::ComputeEigenvectors);
  Eigen::Matrix3d V = solver.eigenvectors();
  // make sure it's a rotation matrix
  V.col(2) = V.col(0).cross(V.col(1));
  const Eigen::Vector3d sigma = solver.eigenvalues().cwiseSqrt() * n_sigma;

  tf::pointEigenToMsg(mu, marker->pose.position);
  tf::quaternionEigenToMsg(Eigen::Quaterniond(V), marker->pose.orientation);
  tf::vectorEigenToMsg(sigma * 2.0, marker->scale);  // diameter, not half axis
  marker->type = visualization_msgs::Marker::SPHERE;
  marker->color = color;
  marker->action = visualization_msgs::Marker::ADD;
}

}  // visualization
}  // namespace orb_slam_2_interface