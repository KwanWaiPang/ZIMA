/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_TRANSFORM_H
#define ZIMA_TRANSFORM_H

#include <map>

#include "zima/common/lock.h"
#include "zima/common/macro.h"
#include "zima/common/maths.h"
#include "zima/common/point_cell.h"

namespace zima {

class Transform {
 public:
  Transform() = delete;
  Transform(const std::string& target_frame, const std::string& source_frame)
      : access_(std::make_shared<ReadWriteLock>()),
        target_frame_(target_frame),
        source_frame_(source_frame),
        x_(0),
        y_(0),
        degree_(0) {}
  Transform(const std::string& target_frame, const std::string& source_frame,
            const double& x, const double& y, const double& degree)
      : access_(std::make_shared<ReadWriteLock>()),
        target_frame_(target_frame),
        source_frame_(source_frame),
        x_(x),
        y_(y),
        degree_(degree) {}

  ~Transform() = default;

  const std::string& SourceFrame() const { return source_frame_; }
  const std::string& TargetFrame() const { return target_frame_; }
  const double& X() const { return x_; }
  const double& Y() const { return y_; }
  const double& Degree() const { return degree_; }

  void X(const double& x) { x_ = x; }
  void Y(const double& y) { y_ = y; }
  void Degree(const double& degree) { degree_ = degree; }

  ReadWriteLock::SPtr GetLock() const { return access_; }

  static void CoordinateTransformationAB(const double& x_in_a,
                                         const double& y_in_a,
                                         const double& x_a_to_b,
                                         const double& y_a_to_b,
                                         const double& radian_a_to_b,
                                         double& x_in_b, double& y_in_b) {
    auto _x = x_in_a - x_a_to_b;
    auto _y = y_in_a - y_a_to_b;
    x_in_b = _x * cos(radian_a_to_b) + _y * sin(radian_a_to_b);
    y_in_b = _y * cos(radian_a_to_b) - _x * sin(radian_a_to_b);
  }

  static void CoordinateTransformationBA(const double& x_in_b,
                                         const double& y_in_b,
                                         const double& x_a_to_b,
                                         const double& y_a_to_b,
                                         const double& radian_a_to_b,
                                         double& x_in_a, double& y_in_a) {
    auto _x = x_in_b * cos(radian_a_to_b) - y_in_b * sin(radian_a_to_b);
    auto _y = y_in_b * cos(radian_a_to_b) + x_in_b * sin(radian_a_to_b);
    x_in_a = _x + x_a_to_b;
    y_in_a = _y + y_a_to_b;
  }

  std::string DebugString() const;

 private:
  ReadWriteLock::SPtr access_;
  std::string target_frame_;
  std::string source_frame_;
  double x_;
  double y_;
  double degree_;
};

class TransformManager {

  DECLARE_SINGLETON(TransformManager)

 public:
  ~TransformManager();

  bool UpdateTransform(const Transform& transform);
  bool UpdateTransform(const std::string& target_frame,
                       const std::string& source_frame,
                       const MapPoint& offset_in_source_frame);
  bool GetTransform(const std::string& target_frame,
                    const std::string& source_frame, Transform& out) const;

  // const std::string kWorldFrame_ = "/world";
  // const std::string kOdomFrame_ = "/odom";
  // const std::string kRobotFrame_ = "/robot";
  // const std::string kLidarFrame_ = "/lidar";
  const std::string kWorldFrame_ = "world";
  const std::string kOdomFrame_ = "odom";
  const std::string kRobotFrame_ = "robot";
  const std::string kLidarFrame_ = "lidar";

 private:
  ReadWriteLock::SPtr access_;
  std::map<std::string, Transform> transforms_;

};

}  // namespace zima

#endif  // ZIMA_TRANSFORM_H
