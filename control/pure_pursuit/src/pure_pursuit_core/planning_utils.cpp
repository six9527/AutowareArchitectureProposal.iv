/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pure_pursuit/util/planning_utils.h"

namespace planning_utils
{
double calcCurvature(const geometry_msgs::Point & target, const geometry_msgs::Pose & current_pose)
{
  constexpr double KAPPA_MAX = 1e9;
  const double radius = calcRadius(target, current_pose);

  if (fabs(radius) > 0) {
    return 1 / radius;
  } else {
    return KAPPA_MAX;
  }
}

double calcDistance2D(const geometry_msgs::Point & p, const geometry_msgs::Point & q)
{
  const double dx = p.x - q.x;
  const double dy = p.y - q.y;
  return sqrt(dx * dx + dy * dy);
}

double calcDistSquared2D(const geometry_msgs::Point & p, const geometry_msgs::Point & q)
{
  const double dx = p.x - q.x;
  const double dy = p.y - q.y;
  return (dx * dx + dy * dy);
}

/* a_vec = line_e - line_s, b_vec = point - line_s
 * a_vec x b_vec = |a_vec| * |b_vec| * sin(theta)
 *               = |a_vec| * lateral_error ( because, lateral_error = |b_vec| * sin(theta) )
 *
 * lateral_error = a_vec x b_vec / |a_vec|
 *        = (a_x * b_y - a_y * b_x) / |a_vec| */
double calcLateralError2D(
  const geometry_msgs::Point & line_s, const geometry_msgs::Point & line_e,
  const geometry_msgs::Point & point)
{
  tf2::Vector3 a_vec((line_e.x - line_s.x), (line_e.y - line_s.y), 0.0);
  tf2::Vector3 b_vec((point.x - line_s.x), (point.y - line_s.y), 0.0);

  double lat_err = (a_vec.length() > 0) ? a_vec.cross(b_vec).z() / a_vec.length() : 0.0;
  return lat_err;
}

double calcRadius(const geometry_msgs::Point & target, const geometry_msgs::Pose & current_pose)
{
  constexpr double RADIUS_MAX = 1e9;
  const double denominator = 2 * transformToRelativeCoordinate2D(target, current_pose).y;
  const double numerator = calcDistSquared2D(target, current_pose.position);

  if (fabs(denominator) > 0)
    return numerator / denominator;
  else
    return RADIUS_MAX;
}

double convertCurvatureToSteeringAngle(double wheel_base, double kappa)
{
  return atan(wheel_base * kappa/2);
}

std::vector<geometry_msgs::Pose> extractPoses(const autoware_planning_msgs::Trajectory & trajectory)
{
  std::vector<geometry_msgs::Pose> poses;

  for (const auto & p : trajectory.points) {
    poses.push_back(p.pose);
  }

  return poses;
}

// get closest point index from current pose
std::pair<bool, int32_t> findClosestIdxWithDistAngThr(
  const std::vector<geometry_msgs::Pose> & poses, const geometry_msgs::Pose & current_pose,
  double th_dist, double th_yaw)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  int32_t idx_min = -1;

  for (size_t i = 0; i < poses.size(); ++i) {
    const double ds = calcDistSquared2D(poses.at(i).position, current_pose.position);
    if (ds > th_dist * th_dist) {
      continue;
    }

    const double yaw_pose = tf2::getYaw(current_pose.orientation);
    const double yaw_ps = tf2::getYaw(poses.at(i).orientation);
    const double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_ps);
    if (fabs(yaw_diff) > th_yaw) {
      continue;
    }

    if (ds < dist_squared_min) {
      dist_squared_min = ds;
      idx_min = i;
    }
  }

  return (idx_min >= 0) ? std::make_pair(true, idx_min) : std::make_pair(false, idx_min);
}

int8_t getLaneDirection(const std::vector<geometry_msgs::Pose> & poses, double th_dist)
{
  if (poses.size() < 2) {
    ROS_ERROR("size of waypoints is smaller than 2");
    return 2;
  }

  for (uint32_t i = 0; i < poses.size(); i++) {
    geometry_msgs::Pose prev;
    geometry_msgs::Pose next;

    if (i == (poses.size() - 1)) {
      prev = poses.at(i - 1);
      next = poses.at(i);
    } else {
      prev = poses.at(i);
      next = poses.at(i + 1);
    }

    if (planning_utils::calcDistSquared2D(prev.position, next.position) > th_dist * th_dist) {
      const auto rel_p = transformToRelativeCoordinate2D(next.position, prev);
      return (rel_p.x > 0.0) ? 0 : 1;
    }
  }

  ROS_ERROR("lane is something wrong");
  return 2;
}

bool isDirectionForward(const geometry_msgs::Pose & prev, const geometry_msgs::Pose & next)
{
  return (transformToRelativeCoordinate2D(next.position, prev).x > 0.0) ? true : false;
}

bool isDirectionForward(const geometry_msgs::Pose & prev, const geometry_msgs::Point & next)
{
  return transformToRelativeCoordinate2D(next, prev).x > 0.0;
}

template <>
bool isInPolygon(
  const std::vector<geometry_msgs::Point> & polygon, const geometry_msgs::Point & point)
{
  std::vector<tf2::Vector3> polygon_conv;
  for (const auto & el : polygon) polygon_conv.emplace_back(el.x, el.y, el.z);

  tf2::Vector3 point_conv = tf2::Vector3(point.x, point.y, point.z);

  return isInPolygon<tf2::Vector3>(polygon_conv, point_conv);
}

double kmph2mps(const double velocity_kmph) { return (velocity_kmph * 1000) / (60 * 60); }

double normalizeEulerAngle(const double euler)
{
  double res = euler;
  while (res > M_PI) {
    res -= (2 * M_PI);
  }
  while (res < -M_PI) {
    res += 2 * M_PI;
  }

  return res;
}

// ref: http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/coordtrans.html
// (pu, pv): relative, (px, py): absolute, (ox, oy): origin
// (px, py) = rot * (pu, pv) + (ox, oy)
geometry_msgs::Point transformToAbsoluteCoordinate2D(
  const geometry_msgs::Point & point, const geometry_msgs::Pose & origin)
{
  // rotation
  geometry_msgs::Point rot_p;
  double yaw = tf2::getYaw(origin.orientation);
  rot_p.x = (cos(yaw) * point.x) + ((-1) * sin(yaw) * point.y);
  rot_p.y = (sin(yaw) * point.x) + (cos(yaw) * point.y);

  // translation
  geometry_msgs::Point res;
  res.x = rot_p.x + origin.position.x;
  res.y = rot_p.y + origin.position.y;
  res.z = origin.position.z;

  return res;
}

// ref: http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/coordtrans.html
// (pu, pv): relative, (px, py): absolute, (ox, oy): origin
// (pu, pv) = rot^-1 * {(px, py) - (ox, oy)}
geometry_msgs::Point transformToRelativeCoordinate2D(
  const geometry_msgs::Point & point, const geometry_msgs::Pose & origin)
{
  // translation
  geometry_msgs::Point trans_p;
  trans_p.x = point.x - origin.position.x;
  trans_p.y = point.y - origin.position.y;

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);

  geometry_msgs::Point res;
  res.x = (cos(yaw) * trans_p.x) + (sin(yaw) * trans_p.y);
  res.y = ((-1) * sin(yaw) * trans_p.x) + (cos(yaw) * trans_p.y);
  res.z = origin.position.z;

  return res;
}

geometry_msgs::Quaternion getQuaternionFromYaw(const double _yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, _yaw);
  return tf2::toMsg(q);
}

}  // namespace planning_utils
