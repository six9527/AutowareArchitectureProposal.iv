/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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
#ifndef DEBUG_OBSTACLEA_AVOIDANCE_PLANNER_H
#define DEBUG_OBSTACLEA_AVOIDANCE_PLANNER_H

struct ConstrainRectangle;
struct Bounds;
struct DebugData;
struct VehicleParam;

namespace util
{
struct Footprint;
}

visualization_msgs::MarkerArray getDebugVisualizationMarker(
  const DebugData & debug_data,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & optimized_points,
  const VehicleParam & vehicle_param);

geometry_msgs::Pose getVirtualWallPose(
  const geometry_msgs::Pose & target_pose, const VehicleParam & vehicle_param);

visualization_msgs::MarkerArray getDebugPointsMarkers(
  const std::vector<geometry_msgs::Point> & interpolated_points,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & optimized_points,
  const std::vector<geometry_msgs::Point> & straight_points,
  const std::vector<geometry_msgs::Pose> & fixed_points,
  const std::vector<geometry_msgs::Pose> & non_fixed_points);

visualization_msgs::MarkerArray getDebugConstrainMarkers(
  const std::vector<ConstrainRectangle> & constrain_ranges, const std::string & ns);

visualization_msgs::MarkerArray getObjectsMarkerArray(
  const std::vector<autoware_perception_msgs::DynamicObject> & objects, const std::string & ns,
  const double r, const double g, const double b);

visualization_msgs::MarkerArray getRectanglesMarkerArray(
  const std::vector<util::Footprint> & rects, const std::string & ns, const double r,
  const double g, const double b);

visualization_msgs::MarkerArray getRectanglesNumMarkerArray(
  const std::vector<util::Footprint> & rects, const std::string & ns, const double r,
  const double g, const double b);

visualization_msgs::MarkerArray getPointsMarkerArray(
  const std::vector<geometry_msgs::Pose> & points, const std::string & ns, const double r,
  const double g, const double b);

visualization_msgs::MarkerArray getPointsMarkerArray(
  const std::vector<geometry_msgs::Point> & points, const std::string & ns, const double r,
  const double g, const double b);

visualization_msgs::MarkerArray getPointsTextMarkerArray(
  const std::vector<geometry_msgs::Pose> & points, const std::string & ns, const double r,
  const double g, const double b);

visualization_msgs::MarkerArray getPointsTextMarkerArray(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points, const std::string & ns,
  const double r, const double g, const double b);

visualization_msgs::MarkerArray getBaseBoundsLineMarkerArray(
  const std::vector<Bounds> & bounds, const std::vector<geometry_msgs::Pose> & candidate_p0,
  const std::string & ns, const double r, const double g, const double b);

visualization_msgs::MarkerArray getTopBoundsLineMarkerArray(
  const std::vector<Bounds> & bounds, const std::vector<geometry_msgs::Pose> & candidate_p1,
  const std::string & ns, const double r, const double g, const double b);

visualization_msgs::MarkerArray getMidBoundsLineMarkerArray(
  const std::vector<Bounds> & bounds, const std::vector<geometry_msgs::Pose> & candidate_top,
  const std::string & ns, const double r, const double g, const double b);

visualization_msgs::MarkerArray getVirtualWallMarkerArray(
  const geometry_msgs::Pose & pose, const std::string & ns, const double r, const double g,
  const double b);

visualization_msgs::MarkerArray getVirtualWallTextMarkerArray(
  const geometry_msgs::Pose & pose, const std::string & ns, const double r, const double g,
  const double b);

nav_msgs::OccupancyGrid getDebugCostmap(
  const cv::Mat & clearance_map, const nav_msgs::OccupancyGrid & occupancy_grid);
#endif
