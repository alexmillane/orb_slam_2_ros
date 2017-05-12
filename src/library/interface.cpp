#include <unistd.h>
#include <thread>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <glog/logging.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "orb_slam_2_ros/KeyframeStatus.h"
#include "orb_slam_2_ros/TransformsWithIds.h"

#include <Eigen/Sparse>

#include "orb_slam_2_ros/TransformsWithIds.h"
#include "orb_slam_2_ros/interface.hpp"

namespace orb_slam_2_interface {

OrbSlam2Interface::OrbSlam2Interface(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      use_viewer_(kDefaultUseViewer),
      verbose_(kDefaultVerbose),
      frame_id_(kDefaultFrameId),
      child_frame_id_(kDefaultChildFrameId),
      mb_shutdown_flag(false) {
  // Getting data and params
  advertiseTopics();
  getParametersFromRos();
}

OrbSlam2Interface::~OrbSlam2Interface() { shutdown(); }

void OrbSlam2Interface::shutdown() {
  // Signaling the shutdown
  {
    std::unique_lock<mutex> lock(m_mutex_shutdown_flag);
    mb_shutdown_flag = true;
  }
}

void OrbSlam2Interface::runCheckForUpdatedTrajectory() {
  // Looping while the interface is alive and checking for loop closures
  // TODO(alexmillane): Should be using condition variables really instead of
  // this polled waiting structure.
  while (!mb_shutdown_flag) {
    // Check if updates to the past trajectory are available
    if (slam_system_->isUpdatedTrajectoryAvailable()) {
      // DEBUG
      std::cout << "Updated trajectory available. Publishing." << std::endl;


      // TODO(alex.millane): Put the below in a function!!!
      // TODO(alex.millane): Include a timestamp!!!


      // Getting the trajectory from the interface
      std::vector<ORB_SLAM2::PoseWithID> T_C_W_trajectory_unnormalized =
          slam_system_->GetUpdatedTrajectory();

      // Publishing the trajectory
      publishUpdatedTrajectory(T_C_W_trajectory_unnormalized);


      // BELOW IS ZACH'S STUFF ON CALCULATING THE MARGINAL COVARIANCE.
      // THIS NEEDS MORE WORK.

/*      // calculating the covaraince
      std::cout << "Calculating the covariance matrix, this will take awhile..."
                << std::endl;
      std::shared_ptr<std::pair<std::vector<size_t>,
                                Eigen::SparseMatrix<double, Eigen::ColMajor>>>
          cov_info = ORB_SLAM2::Optimizer::getCovInfo();

      idxToId_ = cov_info->first;
      idToIdx_.clear();
      for (size_t i = 0; i < idxToId_.size(); ++i) {
        idToIdx_[idxToId_[i]] = i;
      }

      // build up covariance matrix (invert by solving AtA * _covMatrix = I)
      // THIS IS A MASSIVE INEFFICIENT HACK THAT WILL CRIPPLE YOUR RUNTIME
      Eigen::SimplicialLLT<Eigen::SparseMatrix<double, Eigen::ColMajor>>
          llt_solver;
      llt_solver.compute(cov_info->second);
      Eigen::SparseMatrix<double, Eigen::ColMajor> I(cov_info->second.rows(),
                                                     cov_info->second.cols());
      I.setIdentity();
      Eigen::SparseMatrix<double, Eigen::ColMajor> solution =
          llt_solver.solve(I);
      cov_mat_ = solution;*/

    }
    usleep(5000);
  }
}

void OrbSlam2Interface::publishUpdatedTrajectory(
    const std::vector<ORB_SLAM2::PoseWithID>& T_C_W_trajectory) {
  // Populating the trajectory message
  orb_slam_2_ros::TransformsWithIds transforms_with_ids;
  for (const ORB_SLAM2::PoseWithID& pose_with_id : T_C_W_trajectory) {
    // Converting to minkindr
    Transformation T_C_W;
    convertOrbSlamPoseToKindr(pose_with_id.pose, &T_C_W);
    // Inverting for the proper direction
    Transformation T_W_C = T_C_W.inverse();
    // Converting to a transform stamped message
    geometry_msgs::TransformStamped T_W_C_msg;
    T_W_C_msg.header.stamp = ros::Time(pose_with_id.timestamp);
    tf::transformKindrToMsg(T_W_C, &T_W_C_msg.transform);
    // Converting the id to a message
    std_msgs::UInt64 id;
    id.data = pose_with_id.id;
    // Pushing this onto the transform stamped array
    // NOTE(alexmillane): Have stamped this with ros time now because 
    //                    the GBA occurs asynchronously, so this seems
    //                    like the best way to stamp this message.
    transforms_with_ids.header.stamp = ros::Time::now();
    transforms_with_ids.transforms.push_back(T_W_C_msg);
    transforms_with_ids.keyframe_ids.push_back(id);
  }
  // Publishing the trajectory message
  trajectory_pub_.publish(transforms_with_ids);
}

void OrbSlam2Interface::advertiseTopics() {
  // Advertising topics
  T_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
      "transform_cam", 1);
  trajectory_pub_ = nh_private_.advertise<orb_slam_2_ros::TransformsWithIds>(
      "trajectory_cam", 1);
  keyframe_status_pub_ = nh_private_.advertise<orb_slam_2_ros::KeyframeStatus>(
      "keyframe_status", 1);
  // Creating a callback timer for TF publisher
  tf_timer_ = nh_.createTimer(ros::Duration(0.1),
                              &OrbSlam2Interface::publishCurrentPoseAsTF, this);
}

void OrbSlam2Interface::getParametersFromRos() {
  // Getting the paths to the files required by orb slam
  CHECK(nh_private_.getParam("vocabulary_file_path", vocabulary_file_path_))
      << "Please provide the vocabulary_file_path as a ros param.";
  CHECK(nh_private_.getParam("settings_file_path", settings_file_path_))
      << "Please provide the settings_file_path as a ros param.";
  // Optional params
  nh_private_.getParam("use_viewer", use_viewer_);
  nh_private_.getParam("verbose", verbose_);
  nh_private_.getParam("frame_id", frame_id_);
  nh_private_.getParam("child_frame_id", child_frame_id_);
}

void OrbSlam2Interface::publishCurrentPose(const Transformation& T,
                                           const std_msgs::Header& header) {
  // Creating the message
  geometry_msgs::TransformStamped msg;
  // Filling out the header
  // TODO(millane): Should probably just be copying over the time here...
  msg.header = header;
  // Setting the child and parent frames
  msg.child_frame_id = child_frame_id_;
  // Converting from a minkindr transform to a transform message
  tf::transformKindrToMsg(T, &msg.transform);
  // Publishing the current transformation.
  T_pub_.publish(msg);
}

void OrbSlam2Interface::publishCurrentPoseAsTF(const ros::TimerEvent& event) {
  tf::Transform tf_transform;
  tf::transformKindrToTF(T_W_C_, &tf_transform);
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      tf_transform, ros::Time::now(), frame_id_, child_frame_id_));
}

/*void OrbSlam2Interface::publishTrajectory(
    const std::vector<Eigen::Affine3d,
                      Eigen::aligned_allocator<Eigen::Affine3d>>&
        T_C_W_trajectory) {
  // Populating the pose array
  geometry_msgs::PoseArray pose_array_msg;
  for (size_t pose_idx = 0; pose_idx < T_C_W_trajectory.size(); pose_idx++) {
    Eigen::Affine3d T_C_W = T_C_W_trajectory[pose_idx];
    // TODO(alexmillane): This is the wrong place for the inverse. Move it to
    // the extraction function... Also rename the publisher. Gotta go to bed
    // right now.
    Eigen::Affine3d T_W_C = T_C_W.inverse();
    geometry_msgs::Pose pose_msg;
    tf::poseEigenToMsg(T_W_C, pose_msg);
    pose_array_msg.poses.push_back(pose_msg);
  }
  // Publishing
  trajectory_pub_.publish(pose_array_msg);
}*/

void OrbSlam2Interface::convertOrbSlamPoseToKindr(const cv::Mat& T_cv,
                                                  Transformation* T_kindr) {
  // Argument checks
  CHECK_NOTNULL(T_kindr);
  CHECK_EQ(T_cv.cols, 4);
  CHECK_EQ(T_cv.rows, 4);
  // Open CV mat to Eigen matrix (float)
  Eigen::Matrix4f T_eigen_f;
  cv::cv2eigen(T_cv, T_eigen_f);
  // Eigen matrix (float) to Eigen matrix (double)
  Eigen::Matrix4d T_eigen_d = T_eigen_f.cast<double>();
  // Extracting and orthonormalizing the rotation matrix
  Eigen::Matrix3d R_unnormalized = T_eigen_d.block<3, 3>(0, 0);
  Eigen::AngleAxisd aa(R_unnormalized);
  Eigen::Matrix3d R = aa.toRotationMatrix();
  // Constructing the transformation
  Quaternion q_kindr(R);
  Eigen::Vector3d t_kindr(T_eigen_d.block<3, 1>(0, 3));
  *T_kindr = Transformation(q_kindr, t_kindr);
}

void OrbSlam2Interface::publishCurrentKeyframeStatus(
    bool keyframe_status, long unsigned int last_keyframe_id,
    bool big_change_flag, const std_msgs::Header& frame_header) {
  //
  orb_slam_2_ros::KeyframeStatus keyframe_status_msg;
  keyframe_status_msg.keyframe_status = keyframe_status;
  keyframe_status_msg.header.stamp = frame_header.stamp;
  keyframe_status_msg.keyframe_id.data = last_keyframe_id;
  keyframe_status_msg.big_change_status = big_change_flag;
  // Publishing
  keyframe_status_pub_.publish(keyframe_status_msg);
}

bool OrbSlam2Interface::getMarginalUncertainty(int id, Eigen::MatrixXd* cov) {
  if (idToIdx_.find(id) == idToIdx_.end()) {
    return false;
  }
  int idx = idToIdx_[id];

  // get elements
  *cov = cov_mat_.block(idx * 6, idx * 6, 6, 6);

  return true;
}

// I have no idea if this is correct, a straight implementation of this
// wikipedia page (https://en.wikipedia.org/wiki/Schur_complement)
bool OrbSlam2Interface::getJointMarginalUncertainty(int id_x, int id_y,
                                                    Eigen::MatrixXd* cov) {
  if (idToIdx_.find(id_x) == idToIdx_.end()) {
    return false;
  }
  int idx_x = idToIdx_[id_x];

  if (idToIdx_.find(id_y) == idToIdx_.end()) {
    return false;
  }
  int idx_y = idToIdx_[id_y];

  // get parts of shur matrix
  Eigen::Matrix<double, 6, 6> A = cov_mat_.block(idx_x * 6, idx_x * 6, 6, 6);
  Eigen::Matrix<double, 6, 6> B = cov_mat_.block(idx_x * 6, idx_y * 6, 6, 6);
  Eigen::Matrix<double, 6, 6> C = cov_mat_.block(idx_y * 6, idx_y * 6, 6, 6);

  // get covariance
  *cov = A - B * C.inverse() * B.transpose();

  return true;
}

}  // namespace orb_slam_2_interface
