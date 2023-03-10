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

#include <awapi_awiv_adapter/awapi_awiv_adapter_core.h>

namespace autoware_api
{
AutowareIvAdapter::AutowareIvAdapter() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
  // get param
  pnh_.param<double>("status_pub_hz", status_pub_hz_, 5.0);
  pnh_.param<double>("stop_reason_timeout", stop_reason_timeout_, 0.5);
  const double default_max_velocity = waitForParam<double>(pnh_, "param/default_max_velocity");
  pnh_.param<double>("stop_reason_thresh_dist", stop_reason_thresh_dist_, 100.0);
  const bool em_stop_param = waitForParam<bool>(pnh_, "param/emergency_stop");
  emergencyParamCheck(em_stop_param);

  // setup instance
  vehicle_state_publisher_ = std::make_unique<AutowareIvVehicleStatePublisher>();
  autoware_state_publisher_ = std::make_unique<AutowareIvAutowareStatePublisher>();
  stop_reason_aggregator_ = std::make_unique<AutowareIvStopReasonAggregator>(
    stop_reason_timeout_, stop_reason_thresh_dist_);
  lane_change_state_publisher_ = std::make_unique<AutowareIvLaneChangeStatePublisher>();
  obstacle_avoidance_state_publisher_ =
    std::make_unique<AutowareIvObstacleAvoidanceStatePublisher>();
  max_velocity_publisher_ = std::make_unique<AutowareIvMaxVelocityPublisher>(default_max_velocity);

  // publisher
  pub_door_control_ = pnh_.advertise<pacmod_msgs::SystemCmdInt>("output/door_control", 1);
  pub_door_status_ = pnh_.advertise<autoware_api_msgs::DoorStatus>("output/door_status", 1);

  // subscriber
  sub_steer_ = pnh_.subscribe("input/steer", 1, &AutowareIvAdapter::callbackSteer, this);
  sub_vehicle_cmd_ =
    pnh_.subscribe("input/vehicle_cmd", 1, &AutowareIvAdapter::callbackVehicleCmd, this);
  sub_turn_signal_cmd_ =
    pnh_.subscribe("input/turn_signal", 1, &AutowareIvAdapter::callbackTurnSignal, this);
  sub_twist_ = pnh_.subscribe("input/twist", 1, &AutowareIvAdapter::callbackTwist, this);
  sub_gear_ = pnh_.subscribe("input/gear", 1, &AutowareIvAdapter::callbackGear, this);
  sub_battery_ = pnh_.subscribe("input/battery", 1, &AutowareIvAdapter::callbackBattery, this);
  sub_nav_sat_ = pnh_.subscribe("input/nav_sat", 1, &AutowareIvAdapter::callbackNavSat, this);
  sub_autoware_state_ =
    pnh_.subscribe("input/autoware_state", 1, &AutowareIvAdapter::callbackAutowareState, this);
  sub_control_mode_ =
    pnh_.subscribe("input/control_mode", 1, &AutowareIvAdapter::callbackControlMode, this);
  sub_gate_mode_ = pnh_.subscribe("input/gate_mode", 1, &AutowareIvAdapter::callbackGateMode, this);
  sub_emergency_ =
    pnh_.subscribe("input/is_emergency", 1, &AutowareIvAdapter::callbackIsEmergency, this);
  sub_hazard_status_ =
    pnh_.subscribe("input/hazard_status", 1, &AutowareIvAdapter::callbackHazardStatus, this);
  sub_stop_reason_ =
    pnh_.subscribe("input/stop_reason", 100, &AutowareIvAdapter::callbackStopReason, this);
  sub_diagnostics_ =
    pnh_.subscribe("input/diagnostics", 1, &AutowareIvAdapter::callbackDiagnostics, this);
  sub_global_rpt_ =
    pnh_.subscribe("input/global_rpt", 1, &AutowareIvAdapter::callbackGlobalRpt, this);
  sub_lane_change_available_ = pnh_.subscribe(
    "input/lane_change_avaiable", 1, &AutowareIvAdapter::callbackLaneChangeAvailable, this);
  sub_lane_change_ready_ =
    pnh_.subscribe("input/lane_change_ready", 1, &AutowareIvAdapter::callbackLaneChangeReady, this);
  sub_lane_change_candidate_ = pnh_.subscribe(
    "input/lane_change_candidate_path", 1, &AutowareIvAdapter::callbackLaneChangeCandidatePath,
    this);
  sub_obstacle_avoid_ready_ = pnh_.subscribe(
    "input/obstacle_avoid_ready", 1, &AutowareIvAdapter::callbackLaneObstacleAvoidReady, this);
  sub_obstacle_avoid_candidate_ = pnh_.subscribe(
    "input/obstacle_avoid_candidate_path", 1,
    &AutowareIvAdapter::callbackLaneObstacleAvoidCandidatePath, this);
  sub_max_velocity_ =
    pnh_.subscribe("input/max_velocity", 1, &AutowareIvAdapter::callbackMaxVelocity, this);
  sub_current_max_velocity_ = pnh_.subscribe(
    "input/current_max_velocity", 1, &AutowareIvAdapter::callbackCurrentMaxVelocity, this);
  sub_temporary_stop_ =
    pnh_.subscribe("input/temporary_stop", 1, &AutowareIvAdapter::callbackTemporaryStop, this);
  sub_autoware_traj_ = pnh_.subscribe(
    "input/autoware_trajectory", 1, &AutowareIvAdapter::callbackAutowareTrajectory, this);
  sub_door_control_ =
    pnh_.subscribe("input/door_control", 1, &AutowareIvAdapter::callbackDoorControl, this);
  sub_door_status_ =
    pnh_.subscribe("input/door_status", 1, &AutowareIvAdapter::callbackDoorStatus, this);

  // timer
  timer_ =
    nh_.createTimer(ros::Duration(1.0 / status_pub_hz_), &AutowareIvAdapter::timerCallback, this);
}

void AutowareIvAdapter::emergencyParamCheck(const bool emergency_stop_param)
{
  if (!emergency_stop_param) {
    ROS_WARN_STREAM("parameter[use_external_emergency_stop] is false.");
    ROS_WARN_STREAM("autoware/put/emergency is not valid");
  }
}

void AutowareIvAdapter::timerCallback(const ros::TimerEvent & e)
{
  // get current pose
  getCurrentPose();

  // publish vehicle state
  vehicle_state_publisher_->statePublisher(aw_info_);

  // publish autoware state
  autoware_state_publisher_->statePublisher(aw_info_);

  // publish lane change state
  lane_change_state_publisher_->statePublisher(aw_info_);

  // publish obstacle_avoidance state
  obstacle_avoidance_state_publisher_->statePublisher(aw_info_);

  // publish pacmod door status
  pub_door_status_.publish(pacmod_util::getDoorStatusMsg(aw_info_.door_state_ptr));
}

void AutowareIvAdapter::callbackSteer(const autoware_vehicle_msgs::Steering::ConstPtr & msg_ptr)
{
  aw_info_.steer_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackVehicleCmd(
  const autoware_vehicle_msgs::VehicleCommand::ConstPtr & msg_ptr)
{
  aw_info_.vehicle_cmd_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackTurnSignal(
  const autoware_vehicle_msgs::TurnSignal::ConstPtr & msg_ptr)
{
  aw_info_.turn_signal_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr & msg_ptr)
{
  aw_info_.twist_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackGear(const autoware_vehicle_msgs::ShiftStamped::ConstPtr & msg_ptr)
{
  aw_info_.gear_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackBattery(const std_msgs::Float32::ConstPtr & msg_ptr)
{
  aw_info_.battery_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackNavSat(const sensor_msgs::NavSatFix::ConstPtr & msg_ptr)
{
  aw_info_.nav_sat_ptr = msg_ptr;
}

void AutowareIvAdapter::getCurrentPose()
{
  try {
    auto transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    geometry_msgs::PoseStamped ps;
    ps.header = transform.header;
    ps.pose.position.x = transform.transform.translation.x;
    ps.pose.position.y = transform.transform.translation.y;
    ps.pose.position.z = transform.transform.translation.z;
    ps.pose.orientation = transform.transform.rotation;
    aw_info_.current_pose_ptr = std::make_shared<geometry_msgs::PoseStamped>(ps);
  } catch (tf2::TransformException & ex) {
    ROS_DEBUG_STREAM_THROTTLE(2.0, "[awapi_autoware_adapter] cannot get self pose");
  }
}

void AutowareIvAdapter::callbackAutowareState(
  const autoware_system_msgs::AutowareState::ConstPtr & msg_ptr)
{
  aw_info_.autoware_state_ptr = msg_ptr;
}
void AutowareIvAdapter::callbackControlMode(
  const autoware_vehicle_msgs::ControlMode::ConstPtr & msg_ptr)
{
  aw_info_.control_mode_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackGateMode(const autoware_control_msgs::GateMode::ConstPtr & msg_ptr)
{
  aw_info_.gate_mode_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackIsEmergency(const std_msgs::Bool::ConstPtr & msg_ptr)
{
  aw_info_.is_emergency_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackHazardStatus(
  const autoware_system_msgs::HazardStatusStamped::ConstPtr & msg_ptr)
{
  aw_info_.hazard_status_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackStopReason(
  const autoware_planning_msgs::StopReasonArrayConstPtr & msg_ptr)
{
  aw_info_.stop_reason_ptr = stop_reason_aggregator_->updateStopReasonArray(msg_ptr, aw_info_);
}

void AutowareIvAdapter::callbackDiagnostics(
  const diagnostic_msgs::DiagnosticArray::ConstPtr & msg_ptr)
{
  aw_info_.diagnostic_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackGlobalRpt(const pacmod_msgs::GlobalRpt::ConstPtr & msg_ptr)
{
  aw_info_.global_rpt_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackLaneChangeAvailable(const std_msgs::Bool::ConstPtr & msg_ptr)
{
  aw_info_.lane_change_available_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackLaneChangeReady(const std_msgs::Bool::ConstPtr & msg_ptr)
{
  aw_info_.lane_change_ready_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackLaneChangeCandidatePath(
  const autoware_planning_msgs::Path::ConstPtr & msg_ptr)
{
  aw_info_.lane_change_candidate_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackLaneObstacleAvoidReady(const std_msgs::Bool::ConstPtr & msg_ptr)
{
  aw_info_.obstacle_avoid_ready_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackLaneObstacleAvoidCandidatePath(
  const autoware_planning_msgs::Trajectory::ConstPtr & msg_ptr)
{
  aw_info_.obstacle_avoid_candidate_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackMaxVelocity(const std_msgs::Float32::ConstPtr & msg_ptr)
{
  aw_info_.max_velocity_ptr = msg_ptr;
  max_velocity_publisher_->statePublisher(aw_info_);
}

void AutowareIvAdapter::callbackCurrentMaxVelocity(const std_msgs::Float32::ConstPtr & msg_ptr)
{
  aw_info_.current_max_velocity_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackTemporaryStop(const std_msgs::Bool::ConstPtr & msg_ptr)
{
  if (aw_info_.temporary_stop_ptr) {
    if (aw_info_.temporary_stop_ptr->data == msg_ptr->data) {
      //if same value as last time is sent, ignore msg.
      return;
    }
  }

  aw_info_.temporary_stop_ptr = msg_ptr;
  max_velocity_publisher_->statePublisher(aw_info_);
}

void AutowareIvAdapter::callbackAutowareTrajectory(
  const autoware_planning_msgs::Trajectory::ConstPtr & msg_ptr)
{
  aw_info_.autoware_planning_traj_ptr = msg_ptr;
}

void AutowareIvAdapter::callbackDoorControl(const std_msgs::Bool::ConstPtr & msg_ptr)
{
  pub_door_control_.publish(pacmod_util::createClearOverrideDoorCommand());
  ros::Duration(0.1).sleep();  //avoid message loss
  pub_door_control_.publish(pacmod_util::createDoorCommand(msg_ptr));
}

void AutowareIvAdapter::callbackDoorStatus(const pacmod_msgs::SystemRptInt::ConstPtr & msg_ptr)
{
  aw_info_.door_state_ptr = msg_ptr;
}

}  // namespace autoware_api
