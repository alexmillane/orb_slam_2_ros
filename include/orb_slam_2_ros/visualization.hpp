#ifndef ORB_SLAM_2_ROS_VISUALIZATION_H_
#define ORB_SLAM_2_ROS_VISUALIZATION_H_

#include <string>

#include <Eigen/Geometry>

#include <visualization_msgs/MarkerArray.h>

#include <orb_slam_2_ros/types.hpp>

namespace orb_slam_2_interface {
namespace visualization {

// TODO(alexmillane): Probably should be parameters
constexpr double kAxesScale = 1.0;
constexpr double kAxesDiameter = 0.5;
constexpr double kAxesAlpha = 1.0;

constexpr double kPatchAxesScale =3.0;
constexpr double kPatchAxesDiameter = 1.0;
constexpr double kPatchAxesAlpha = 1.0;

constexpr double kKeyframeColors[3][3] = {
    {1, 0, 0},  // red
    {0, 1, 0},  // blue
    {0, 0, 1}   // green
};

constexpr double kFrameColors[3][3] = {
    {192, 192, 192},  // grey
    {192, 192, 192},  // grey
    {192, 192, 192}   // grey
};

constexpr double kOptimizedKeyframeColors[3][3] = {
    {255, 255, 0},  // redish
    {0, 255, 255},  // blueish
    {255, 0, 255}   // greenish
};

constexpr double kOptimizedPatchframeColors[3][3] = {
    {255, 255, 0},  // redish
    {0, 255, 255},  // blueish
    {255, 0, 255}   // greenish
};

constexpr double kKeyframeCovarianceColor[3] = {255, 255, 0};
constexpr double kKeyframeCovarianceAlpha = 0.25;
constexpr double kKeyframeCovarianceScale = 5;

constexpr double kPatchCovarianceColor[3] = {0, 255, 255};
constexpr double kPatchCovarianceAlpha = 0.5;
constexpr double kPatchCovarianceScale = 25;

enum class FrameType { Frame, KeyFrame, OptimizedKeyFrame, OptimizedPatchFrame };
enum class CovarianceType { OptimizedKeyFrame, OptimizedPatchFrame };

void addFrameToMarkerArray(const Transformation& T_M_C,
                           const std::string& frame,
                           const FrameType& frame_type,
                           visualization_msgs::MarkerArray* marker_array_ptr,
                           size_t* axes_marker_index_ptr);

//void getKeyFrameMarkerArray

inline std_msgs::ColorRGBA createColorRGBA(float r, float g, float b, float a);

void drawArrowPoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                     const std_msgs::ColorRGBA& color, double diameter,
                     visualization_msgs::Marker* marker);

void drawAxesArrowsWithColor(const Eigen::Vector3d& p,
                             const Eigen::Quaterniond& q, double scale,
                             double diameter, const double colors[3][3], double alpha,
                             std::vector<visualization_msgs::Marker>* markers);

void addCovarianceEllipseToMarkerArray(
    const Transformation& T_M_C, const Eigen::Matrix3d& covariance,
    const std::string& frame, const CovarianceType& covariance_type,
    visualization_msgs::MarkerArray* marker_array_ptr,
    size_t* axes_marker_index_ptr);

/**
 * \brief Draws a covariance ellipsoid
 * \param[out] marker The marker in which the ellipsoid should be drawn
 * \param[in] mu static 3 element vector, specifying the center of the ellipsoid
 * \param[in] cov static 3x3 covariance matrix
 * \param[in] color RGBA color of the ellipsoid
 * \param[in] n_sigma confidence area / scale of the ellipsoid
 */
/*void drawCovariance3D(const Eigen::Vector3d& mu, const Eigen::Matrix3d& cov,
                      const std_msgs::ColorRGBA& color, double n_sigma,
                      visualization_msgs::Marker* marker);
*/

void drawCovariance3D(const Eigen::Vector3d& mu, const Eigen::Matrix3d& cov,
                      const double colors[3], double alpha, double n_sigma,
                      visualization_msgs::Marker* marker);

}  // namespace visualization
}  // namespace orb_slam_2_interface

#endif /* ORB_SLAM_2_ROS_VISUALIZATION_H_ */
