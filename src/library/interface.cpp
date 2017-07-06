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
#include <Eigen/StdVector>

#include "orb_slam_2_ros/TransformsWithIds.h"
#include "orb_slam_2_ros/interface.hpp"
#include "orb_slam_2_ros/visualization.hpp"
#include "orb_slam_2_ros/matrix_io.hpp"

namespace orb_slam_2_interface {

OrbSlam2Interface::OrbSlam2Interface(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      use_viewer_(kDefaultUseViewer),
      verbose_(kDefaultVerbose),
      frame_id_(kDefaultFrameId),
      child_frame_id_(kDefaultChildFrameId),
      mb_shutdown_flag(false),
      frame_marker_counter_(0) {
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

void OrbSlam2Interface::advertiseTopics() {
  // Advertising topics
  T_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
      "transform_cam", 1);
  trajectory_pub_ = nh_private_.advertise<orb_slam_2_ros::TransformsWithIds>(
      "trajectory_cam", 1);
  keyframe_status_pub_ = nh_private_.advertise<orb_slam_2_ros::KeyframeStatus>(
      "keyframe_status", 1);
  // Advertising the visualization topics
  frame_visualization_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "frame_visualization", 1, true);
  optimized_frame_visualization_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "optimized_frame_visualization", 1, true);
  // Advertising services
  global_bundle_adjustment_srv_ = nh_private_.advertiseService(
      "start_global_bundle_adjustment",
      &OrbSlam2Interface::startGlobalBundleAdjustmentCallback, this);
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

bool OrbSlam2Interface::startGlobalBundleAdjustmentCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  // DEBUG
  std::cout << "In the start global bundle adjustment service." << std::endl;
  // Kicking off bundle adjustment
  return slam_system_->startGlobalBundleAdjustment();
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
      // Publishing the optimized trajectory
      publishUpdatedTrajectoryVisualization(T_C_W_trajectory_unnormalized);

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

void OrbSlam2Interface::publishCurrentPoseVisualization(const Transformation& T,
                                                        bool keyframe_flag) {
  // The frame type
  visualization::FrameType frame_type;
  if (keyframe_flag == true) {
    frame_type = visualization::FrameType::KeyFrame;
  } else {
    frame_type = visualization::FrameType::Frame;
  }
  // Getting the marker array
  visualization_msgs::MarkerArray marker_array;
  visualization::addFrameToMarkerArray(T, frame_id_, frame_type,
                                       &marker_array, &frame_marker_counter_);
  // Publishing
  frame_visualization_pub_.publish(marker_array);
}

void OrbSlam2Interface::publishUpdatedTrajectoryVisualization(
    const std::vector<ORB_SLAM2::PoseWithID>& T_C_W_trajectory) {

  // DEBUG
  std::cout << "Publishing the optimized trajectory visualizations" << std::endl;

  // Looping over the new keyframe positions and publishing the visualizations
  visualization_msgs::MarkerArray marker_array;
  size_t optimized_frame_marker_counter = 0;
  static const std::string ns = "optimized_keyframe";
  static const auto frame_type = visualization::FrameType::OptimizedKeyFrame;
  for (const ORB_SLAM2::PoseWithID& pose_with_id : T_C_W_trajectory) {
    // Converting to minkindr
    Transformation T_C_W;
    convertOrbSlamPoseToKindr(pose_with_id.pose, &T_C_W);
    // Inverting for the proper direction
    Transformation T_W_C = T_C_W.inverse();
    // Getting the marker array
    visualization::addFrameToMarkerArray(T_W_C, frame_id_, frame_type,
                                         &marker_array,
                                         &optimized_frame_marker_counter);
    // Adding the covariances if available
    if (pose_with_id.covarianceValid) {
      // Getting the position covariance
      Eigen::Matrix3d position_covariance_C =
          pose_with_id.covariance.block<3, 3>(3, 3);
      // Adding this to the marker array
      static const auto covariance_type = visualization::CovarianceType::OptimizedKeyFrame;
      visualization::addCovarianceEllipseToMarkerArray(
          T_W_C, position_covariance_C, frame_id_, covariance_type,
          &marker_array, &optimized_frame_marker_counter);
    }
  }

  // Publishing the patch base frames 
  constexpr bool publishPatchBaseFramesVisualizations = true;
  if (publishPatchBaseFramesVisualizations) {

    // THIS IS JUST TEMPORARY TESTING OF THE MARGINAL COVARIANCE FUNCTIONS.
    // ADDING ARBITRARILY PATCH BASE FRAMES
    {
      // Clearing the keyframes
      slam_system_->removeAllKeyframesAsPatchBaseFrames();

      // Adding some test base frames.
      std::vector<unsigned long> keyframe_ids;
      slam_system_->getCurrentKeyframeIds(&keyframe_ids);

      // FOR TESTING
      // Adding every 10th keyframe as a patch base frame
      constexpr int kfs_per_patch = 10;
      for (size_t KFidx = 0; KFidx < keyframe_ids.size();
           KFidx += kfs_per_patch) {
        slam_system_->addKeyframeAsPatchBaseframe(keyframe_ids[KFidx]);
      }
    }

    // Getting patch poses and covariances
    std::vector<cv::Mat> patch_poses;
    std::vector<Eigen::Matrix<double, 6, 6>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>>
        patch_conditional_covariances;
    slam_system_->getPatchBaseFramePosesAndCovariances(
        &patch_poses, &patch_conditional_covariances);

    // DEBUG
    std::cout << "patch_poses.size(): " << patch_poses.size() << std::endl;
    std::cout << "patch_conditional_covariances.size(): "
              << patch_conditional_covariances.size() << std::endl;

    // Visualizing the patch base frames
    static const std::string ns = "optimized_patchframes";
    static const auto frame_type =
        visualization::FrameType::OptimizedPatchFrame;
    // for (const cv::Mat& patch_pose : patch_poses) {
    for (size_t patch_idx = 0; patch_idx < patch_poses.size(); patch_idx++) {
      // Getting the pose
      const cv::Mat patch_pose = patch_poses[patch_idx];
      // Converting to minkindr
      Transformation T_P_W;
      convertOrbSlamPoseToKindr(patch_pose, &T_P_W);
      // Inverting for the proper direction
      Transformation T_W_P = T_P_W.inverse();
      // Getting the marker array
      visualization::addFrameToMarkerArray(T_W_P, frame_id_, frame_type,
                                           &marker_array,
                                           &optimized_frame_marker_counter);
      // NOTE(alexmillane): The reason we start at patch 2
      // First patch:   Does not have a conditional covariance
      // Second patch:  The conditional covariance appears to be numerically
      //                unstable because the covariance of the first keyframe is tiny.
      if (patch_idx > 1) {
        // DEBUG
        std::cout << "marginal ovariance of patch: " << patch_idx << std::endl
                  << patch_conditional_covariances[patch_idx - 1] << std::endl;

        // Getting the position covariance
        Eigen::Matrix3d patch_position_covariance_C =
            patch_conditional_covariances[patch_idx - 1].block<3, 3>(3, 3);
        // Adding this to the marker array
        static const auto covariance_type =
            visualization::CovarianceType::OptimizedPatchFrame;
        visualization::addCovarianceEllipseToMarkerArray(
            T_W_P, patch_position_covariance_C, frame_id_, covariance_type,
            &marker_array, &optimized_frame_marker_counter);
      }
    }

    // Combine the conditional covariances to a single matrix for output
    std::cout << "Saving the combined conditional covariance matrix to file"
              << std::endl;
    Eigen::MatrixXd combined_coditional_covariance((patch_poses.size() - 2) * 6,
                                                   6);
    for (size_t patch_idx = 2; patch_idx < patch_poses.size(); patch_idx++) {
      combined_coditional_covariance.block<6, 6>((patch_idx - 2) * 6, 0) =
          patch_conditional_covariances[patch_idx - 1];
    }
    // Saving the conditional covariances as a single matrix
    static const std::string combined_conditional_covariance_file_path =
        "/home/millanea/trunk/manifold_mapping_analysis/data/orb_slam/"
        "covariance/combined_conditional_covariances";
    io::writeMatlab(combined_conditional_covariance_file_path.c_str(),
                    combined_coditional_covariance);
  }

  // Publishing
  optimized_frame_visualization_pub_.publish(marker_array);
}

}  // namespace orb_slam_2_interface
