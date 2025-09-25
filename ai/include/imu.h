#pragma once

#include "realsenseloader.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen3/Eigen/src/Core/Matrix.h>

namespace drone {
static Eigen::Matrix3f rotationMatrixFromVector(const Eigen::Vector3f &v,
                                                const Eigen::Vector3f &u) {
  auto v_nrm = v.normalized();
  auto u_nrm = u.normalized();
  auto orth = v_nrm.cross(u_nrm);
  auto s = orth.norm();
  auto c = v_nrm.dot(u_nrm);
  Eigen::Matrix3f K;
  K << 0, -orth(2), orth(1), orth(2), 0, -orth(0), -orth(1), orth(0), 0;
  Eigen::Matrix3f R;
  assert(s != 0);
  return Eigen::Matrix3f::Identity() + K + K * K * ((1 - c) / (s * s));
}

static const Eigen::Vector3f GRAVITY(0.0, 0.0, 9.81f);
class RotationEstimator {
public:
  RotationEstimator() : last_gyro_time_(0.0) {};
  void update(const PosePair &gyro) {
    const auto &[gyro_data, gyro_time] = gyro;
    auto gyro_time_d = (gyro_time - last_gyro_time_) / 1e9;
    last_gyro_time_ = gyro_time;
    theta = gyro_data * gyro_time_d;
  }
  void calibrate(const PosePair &gyro) {
    last_gyro_time_ = gyro.second;
    auto R = rotationMatrixFromVector(gyro.first, GRAVITY);
    Eigen::AngleAxisf angle_axis(R);
    theta = angle_axis.angle() * angle_axis.axis();
  }
  Eigen::Matrix3f getRotation() const {
    float angle_norm = theta.norm();
    Eigen::AngleAxisf angle_axis(angle_norm, theta.normalized());
    return angle_axis.toRotationMatrix().transpose();
  }

private:
  double last_gyro_time_;
  Eigen::Vector3f theta = Eigen::Vector3f::Zero();
};

class TranslationEstimator {
public:
  TranslationEstimator() : last_accel_time_(0.0), gravity_(GRAVITY) {};
  void calibrate(const PosePair &accel) { last_accel_time_ = accel.second; }
  void update(const PosePair &accel, const Eigen::Matrix3f &R) {
    const auto &[accel_data, accel_time] = accel;
    auto accel_time_d = (accel_time - last_accel_time_) / 1e9;
    last_accel_time_ = accel_time;
    auto linear_accel = R.transpose() * (accel_data - R * gravity_);
    t_ += 0.5 * linear_accel * accel_time_d * accel_time_d;
  }
  Eigen::Vector3f getTranslation() const { return t_; }

private:
  double last_accel_time_;
  Eigen::Vector3f gravity_;
  Eigen::Vector3f t_ = Eigen::Vector3f::Zero();
};

template <typename R = RotationEstimator, typename T = TranslationEstimator>
class ImuPoseEstimator {
public:
  void updateMotion(const PosePair &gyro, const PosePair &accel) {
    rotation_estimator_.update(gyro);
    auto rot = rotation_estimator_.getRotation();
    translation_estimator_.update(accel, rot);
    auto t = translation_estimator_.getTranslation();
    pose_.rotate(rot);
    pose_.pretranslate(t);
  }

  void calibrate(const PosePair &gyro, const PosePair &accel) {
    rotation_estimator_.calibrate(gyro);
    translation_estimator_.calibrate(accel);
  }

  const Eigen::Isometry3f &getPose() const { return pose_; }

private:
  RotationEstimator rotation_estimator_;
  TranslationEstimator translation_estimator_;
  Eigen::Isometry3f pose_ = Eigen::Isometry3f::Identity();
};
}; // namespace drone
