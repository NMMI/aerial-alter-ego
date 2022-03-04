// Author: Enrico Corvaglia
// skew_symmetric() takes a vector as input and apply it the skew-symmetric operator
// returns the related skew-symmetric matrix

#ifndef SKEW_SYMMETRIC_H
#define SKEW_SYMMETRIC_H

#include <kdl/kdl.hpp>
#include <Eigen/Core>

inline void skew_symmetric(Eigen::Vector3d &v_, Eigen::Matrix3d &skew_mat_)
{
  skew_mat_ = Eigen::Matrix3d::Zero();

  skew_mat_(0, 1) = -v_(2);
  skew_mat_(0, 2) = v_(1);
  skew_mat_(1, 0) = v_(2);
  skew_mat_(1, 2) = -v_(0);
  skew_mat_(2, 0) = -v_(1);
  skew_mat_(2, 1) = v_(0);
}

#endif