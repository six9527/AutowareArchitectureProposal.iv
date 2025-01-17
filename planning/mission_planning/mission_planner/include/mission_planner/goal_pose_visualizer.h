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

#ifndef MISSION_PLANNER_GOAL_POSE_VISUALIZER_H
#define MISSION_PLANNER_GOAL_POSE_VISUALIZER_H

#include <ros/ros.h>

#include <autoware_planning_msgs/Route.h>
#include <geometry_msgs/PoseStamped.h>

namespace mission_planner
{
class GoalPoseVisualizer
{
public:
  GoalPoseVisualizer();

private:
  ros::NodeHandle pnh_{"~"};

  ros::Subscriber sub_route_;
  ros::Publisher pub_goal_pose_;

  void echoBackRouteCallback(const autoware_planning_msgs::RouteConstPtr & msg);
};

}  // namespace mission_planner
#endif  // MISSION_PLANNER_GOAL_POSE_VISUALIZER_H
