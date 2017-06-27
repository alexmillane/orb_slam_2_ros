#ifndef ORB_SLAM_2_ROS_INTERFACE_H_
#define ORB_SLAM_2_ROS_INTERFACE_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <std_srvs/Empty.h>

#include <geometry_msgs/TransformStamped.h>
#include <orb_slam_2/Optimizer.h>
#include <orb_slam_2/System.h>
//#include <orb_slam_2/DenseMappingInterface.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>

#include "orb_slam_2_ros/types.hpp"

namespace orb_slam_2_interface {

// Default values for parameters
static const bool kDefaultUseViewer = true;
static const bool kDefaultVerbose = true;
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultChildFrameId = "cam0";

// Class handling global alignment calculation and publishing
class OrbSlam2Interface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  OrbSlam2Interface(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);
  ~OrbSlam2Interface();

 protected:
  // Shutsdown the interface
  void shutdown();

  // Subscribes and Advertises to the appropriate ROS topics
  void advertiseTopics();
  void getParametersFromRos();

  // Callbacks
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // Service callbacks
  bool startGlobalBundleAdjustmentCallback(std_srvs::Empty::Request& request,
                                           std_srvs::Empty::Response& response);

  // Contains a while loop that checks for updates to the past trajectories and
  // then publishes them.
  void runCheckForUpdatedTrajectory();

  // Pose Publishing functions
  void publishCurrentPose(const Transformation& T,
                          const std_msgs::Header& header);
  void publishCurrentPoseAsTF(const ros::TimerEvent& event);
  void publishCurrentKeyframeStatus(bool keyframe_status,
                                    long unsigned int last_keyframe_id,
                                    bool big_change_flag,
                                    const std_msgs::Header& frame_header);
  void publishUpdatedTrajectory(
      const std::vector<ORB_SLAM2::PoseWithID>& T_C_W_trajectory);


  // Visualization functions
  void publishCurrentPoseVisualization(const Transformation& T,
                                       bool keyframe_flag);
  void publishUpdatedTrajectoryVisualization(
      const std::vector<ORB_SLAM2::PoseWithID>& T_C_W_trajectory);

  // Helper functions
  void convertOrbSlamPoseToKindr(const cv::Mat& T_cv, Transformation* T_kindr);

  bool getMarginalUncertainty(int id, Eigen::MatrixXd* cov);
  bool getJointMarginalUncertainty(int id_x, int id_y, Eigen::MatrixXd* cov);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers
  ros::Publisher T_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Timer tf_timer_;
  ros::Publisher trajectory_pub_;
  ros::Publisher keyframe_status_pub_;
  ros::Publisher frame_visualization_pub_;
  ros::Publisher optimized_frame_visualization_pub_;

  // Services
  ros::ServiceServer global_bundle_adjustment_srv_;

  // Pointer to the thread that checks for and publishes loop closures
  std::thread* mpt_loop_closure_publisher;

  // The orb slam system
  std::shared_ptr<ORB_SLAM2::System> slam_system_;

  // The current pose
  Transformation T_W_C_;

  // which keyframe id goes with each row of covariance matrix
  std::vector<long unsigned int> idxToId_;
  std::map<long unsigned int, size_t> idToIdx_;

  // The current covariance matrix
  Eigen::MatrixXd cov_mat_;

  // Parameters
  bool use_viewer_;
  bool verbose_;
  std::string vocabulary_file_path_;
  std::string settings_file_path_;

  // Transform frame names
  std::string frame_id_;
  std::string child_frame_id_;

  // A counter for the number of published frame markers
  size_t frame_marker_counter_;

  // Signaling members
  std::mutex m_mutex_shutdown_flag;
  bool mb_shutdown_flag;
};

}  // namespace orb_slam_2_interface

#endif /* ORB_SLAM_2_ROS_INTERFACE_H_ */
