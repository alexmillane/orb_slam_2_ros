#ifndef ORB_SLAM_2_ROS_MATRIX_IO_H_
#define ORB_SLAM_2_ROS_MATRIX_IO_H_

#include <fstream>
#include <iostream>
#include <iomanip>

#include <Eigen/Geometry>

namespace orb_slam_2_interface {
namespace io {

bool writeMatlab(const char* filename, const Eigen::MatrixXd& mat) {
  std::string name = filename;
  std::string::size_type lastDot = name.find_last_of('.');
  if (lastDot != std::string::npos) 
    name = name.substr(0, lastDot);

  std::ofstream fout(filename);
  fout << std::setprecision(9) << std::fixed;

  // Writing to the file.
  fout << mat;

  return fout.good();
}

} // end namespace
} // end namespace

#endif
