/*
 *  Copyright (c) 2017, Tier IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "vehicle_cmd_gate/vehicle_cmd_gate.h"

namespace
{
void fillFrameId(std::string * frame_id, const std::string & name)
{
  if (*frame_id == "") {
    *frame_id = name;
  }
}

const char * getGateModeName(const autoware_control_msgs::GateMode::_data_type & gate_mode)
{
  using autoware_control_msgs::GateMode;

  if (gate_mode == GateMode::AUTO) return "AUTO";
  if (gate_mode == GateMode::REMOTE) return "REMOTE";

  return "NOT_SUPPORTED";
}

bool isHeartbeatTimeout(
  const std::shared_ptr<ros::Time> & heartbeat_received_time, const double timeout)
{
  if (timeout == 0.0) {
    return false;
  }

  if (!heartbeat_received_time) {
    return true;
  }

  const auto time_from_heartbeat = ros::Time::now() - *heartbeat_received_time;

  return time_from_heartbeat.toSec() > timeout;
}
}  // namespace

VehicleCmdGate::VehicleCmdGate()
{
  // Publisher
  vehicle_cmd_pub_ =
    pnh_.advertise<autoware_vehicle_msgs::VehicleCommand>("output/vehicle_cmd", 1, true);
  control_cmd_pub_ =
    pnh_.advertise<autoware_control_msgs::ControlCommandStamped>("output/control_cmd", 1, true);
  shift_cmd_pub_ = pnh_.advertise<autoware_vehicle_msgs::ShiftStamped>("output/shift_cmd", 1, true);
  turn_signal_cmd_pub_ =
    pnh_.advertise<autoware_vehicle_msgs::TurnSignal>("output/turn_signal_cmd", 1, true);
  gate_mode_pub_ = pnh_.advertise<autoware_control_msgs::GateMode>("output/gate_mode", 1, true);

  // Subscriber
  engage_sub_ = pnh_.subscribe("input/engage", 1, &VehicleCmdGate::onEngage, this);
  system_emergency_sub_ =
    pnh_.subscribe("input/system_emergency", 1, &VehicleCmdGate::onSystemEmergency, this);
  external_emergency_stop_sub_ = pnh_.subscribe(
    "input/external_emergency_stop", 1, &VehicleCmdGate::onExternalEmergencyStop, this);
  gate_mode_sub_ = pnh_.subscribe("input/gate_mode", 1, &VehicleCmdGate::onGateMode, this);
  steer_sub_ = pnh_.subscribe("input/steering", 1, &VehicleCmdGate::onSteering, this);

  // Subscriber for auto
  auto_control_cmd_sub_ =
    pnh_.subscribe("input/auto/control_cmd", 1, &VehicleCmdGate::onAutoCtrlCmd, this);
  auto_turn_signal_cmd_sub_ =
    pnh_.subscribe("input/auto/turn_signal_cmd", 1, &VehicleCmdGate::onAutoTurnSignalCmd, this);
  auto_shift_cmd_sub_ =
    pnh_.subscribe("input/auto/shift_cmd", 1, &VehicleCmdGate::onAutoShiftCmd, this);

  // Subscriber for remote
  remote_control_cmd_sub_ =
    pnh_.subscribe("input/remote/control_cmd", 1, &VehicleCmdGate::onRemoteCtrlCmd, this);
  remote_turn_signal_cmd_sub_ =
    pnh_.subscribe("input/remote/turn_signal_cmd", 1, &VehicleCmdGate::onRemoteTurnSignalCmd, this);
  remote_shift_cmd_sub_ =
    pnh_.subscribe("input/remote/shift_cmd", 1, &VehicleCmdGate::onRemoteShiftCmd, this);

  // Subscriber for emergency
  emergency_control_cmd_sub_ =
    pnh_.subscribe("input/emergency/control_cmd", 1, &VehicleCmdGate::onEmergencyCtrlCmd, this);
  emergency_turn_signal_cmd_sub_ = pnh_.subscribe(
    "input/emergency/turn_signal_cmd", 1, &VehicleCmdGate::onEmergencyTurnSignalCmd, this);
  emergency_shift_cmd_sub_ =
    pnh_.subscribe("input/emergency/shift_cmd", 1, &VehicleCmdGate::onEmergencyShiftCmd, this);

  // Parameter
  pnh_.param("update_rate", update_rate_, 10.0);
  pnh_.param("use_emergency_handling", use_emergency_handling_, false);
  pnh_.param("use_external_emergency_stop", use_external_emergency_stop_, false);
  pnh_.param("system_emergency_heartbeat_timeout", system_emergency_heartbeat_timeout_, 0.5);
  pnh_.param(
    "external_emergency_stop_heartbeat_timeout", external_emergency_stop_heartbeat_timeout_, 0.5);

  // Vehicle Parameter
  double wheel_base, vel_lim, lon_acc_lim, lon_jerk_lim, lat_acc_lim, lat_jerk_lim;
  pnh_.param<double>("/vehicle_info/wheel_base", wheel_base, 2.79);
  pnh_.param<double>("vel_lim", vel_lim, 25.0);
  pnh_.param<double>("lon_acc_lim", lon_acc_lim, 5.0);
  pnh_.param<double>("lon_jerk_lim", lon_jerk_lim, 5.0);
  pnh_.param<double>("lat_acc_lim", lat_acc_lim, 5.0);
  pnh_.param<double>("lat_jerk_lim", lat_jerk_lim, 5.0);
  filter_.setWheelBase(wheel_base);
  filter_.setVelLim(vel_lim);
  filter_.setLonAccLim(lon_acc_lim);
  filter_.setLonJerkLim(lon_jerk_lim);
  filter_.setLatAccLim(lat_acc_lim);
  filter_.setLatJerkLim(lat_jerk_lim);

  // Set default value
  current_gate_mode_.data = autoware_control_msgs::GateMode::AUTO;

  // Service
  srv_external_emergency_stop_ = pnh_.advertiseService(
    "service/external_emergency_stop", &VehicleCmdGate::onExternalEmergencyStopService, this);
  srv_clear_external_emergency_stop_ = pnh_.advertiseService(
    "service/clear_external_emergency_stop", &VehicleCmdGate::onClearExternalEmergencyStopService,
    this);

  // Diagnostics Updater
  updater_.setHardwareID("vehicle_cmd_gate");
  updater_.add(
    "heartbeat", [](auto & stat) { stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Alive"); });
  updater_.add("emergency_stop_operation", this, &VehicleCmdGate::checkExternalEmergencyStop);

  // Timer
  timer_ = pnh_.createTimer(ros::Rate(update_rate_), &VehicleCmdGate::onTimer, this);
}

// for auto
void VehicleCmdGate::onAutoCtrlCmd(
  const autoware_control_msgs::ControlCommandStamped::ConstPtr & msg)
{
  auto_commands_.control = *msg;

  if (current_gate_mode_.data == autoware_control_msgs::GateMode::AUTO) {
    publishControlCommands(auto_commands_);
  }
}
void VehicleCmdGate::onAutoTurnSignalCmd(const autoware_vehicle_msgs::TurnSignal::ConstPtr & msg)
{
  auto_commands_.turn_signal = *msg;
}
void VehicleCmdGate::onAutoShiftCmd(const autoware_vehicle_msgs::ShiftStamped::ConstPtr & msg)
{
  auto_commands_.shift = *msg;
}

// for remote
void VehicleCmdGate::onRemoteCtrlCmd(
  const autoware_control_msgs::ControlCommandStamped::ConstPtr & msg)
{
  remote_commands_.control = *msg;

  if (current_gate_mode_.data == autoware_control_msgs::GateMode::REMOTE) {
    publishControlCommands(remote_commands_);
  }
}
void VehicleCmdGate::onRemoteTurnSignalCmd(const autoware_vehicle_msgs::TurnSignal::ConstPtr & msg)
{
  remote_commands_.turn_signal = *msg;
}
void VehicleCmdGate::onRemoteShiftCmd(const autoware_vehicle_msgs::ShiftStamped::ConstPtr & msg)
{
  remote_commands_.shift = *msg;
}

// for emergency
void VehicleCmdGate::onEmergencyCtrlCmd(
  const autoware_control_msgs::ControlCommandStamped::ConstPtr & msg)
{
  emergency_commands_.control = *msg;

  if (use_emergency_handling_ && is_system_emergency_) {
    publishControlCommands(emergency_commands_);
  }
}
void VehicleCmdGate::onEmergencyTurnSignalCmd(
  const autoware_vehicle_msgs::TurnSignal::ConstPtr & msg)
{
  emergency_commands_.turn_signal = *msg;
}
void VehicleCmdGate::onEmergencyShiftCmd(const autoware_vehicle_msgs::ShiftStamped::ConstPtr & msg)
{
  emergency_commands_.shift = *msg;
}

void VehicleCmdGate::onTimer(const ros::TimerEvent & event)
{
  updater_.force_update();

  // Check system emergency heartbeat
  if (use_emergency_handling_) {
    is_system_emergency_heartbeat_timeout_ = isHeartbeatTimeout(
      system_emergency_heartbeat_received_time_, system_emergency_heartbeat_timeout_);

    if (is_system_emergency_heartbeat_timeout_) {
      ROS_WARN_THROTTLE(1.0, "system_emergency heartbeat is timeout.");
      publishEmergencyStopControlCommands();
      return;
    }
  }

  // Check external emergency stop heartbeat
  if (use_external_emergency_stop_) {
    is_external_emergency_stop_heartbeat_timeout_ = isHeartbeatTimeout(
      external_emergency_stop_heartbeat_received_time_, external_emergency_stop_heartbeat_timeout_);

    if (is_external_emergency_stop_heartbeat_timeout_) {
      ROS_WARN_THROTTLE(1.0, "external_emergency_stop heartbeat is timeout.");
      is_external_emergency_stop_ = true;
    }
  }

  // Check external emergency stop
  if (is_external_emergency_stop_) {
    if (!is_external_emergency_stop_heartbeat_timeout_) {
      ROS_INFO_THROTTLE(1.0, "Please call `clear_external_emergency_stop` service to clear state.");
    }

    publishEmergencyStopControlCommands();
    return;
  }

  // Select commands
  autoware_vehicle_msgs::TurnSignal turn_signal;
  autoware_vehicle_msgs::ShiftStamped shift;
  if (use_emergency_handling_ && is_system_emergency_) {
    turn_signal = emergency_commands_.turn_signal;
    shift = emergency_commands_.shift;
  } else {
    if (current_gate_mode_.data == autoware_control_msgs::GateMode::AUTO) {
      turn_signal = auto_commands_.turn_signal;
      shift = auto_commands_.shift;

      // Don't send turn signal when autoware is not engaged
      if (!is_engaged_) {
        turn_signal.data = autoware_vehicle_msgs::TurnSignal::NONE;
      }
    } else if (current_gate_mode_.data == autoware_control_msgs::GateMode::REMOTE) {
      turn_signal = remote_commands_.turn_signal;
      shift = remote_commands_.shift;
    } else {
      throw std::runtime_error("invalid mode");
    }
  }

  // Add frame_id to prevent RViz warnings
  fillFrameId(&shift.header.frame_id, "base_link");
  fillFrameId(&turn_signal.header.frame_id, "base_link");

  // Publish topics
  gate_mode_pub_.publish(current_gate_mode_);
  turn_signal_cmd_pub_.publish(turn_signal);
  shift_cmd_pub_.publish(shift);
}

void VehicleCmdGate::publishControlCommands(const Commands & commands)
{
  // Check system emergency
  if (use_emergency_handling_ && is_system_emergency_heartbeat_timeout_) {
    return;
  }

  // Check external emergency stop
  if (is_external_emergency_stop_) {
    return;
  }

  Commands filtered_commands;

  // Set default commands
  {
    filtered_commands.control = commands.control;
    filtered_commands.shift = commands.shift;  // tmp
  }

  // Check system emergency
  if (use_emergency_handling_ && is_system_emergency_) {
    ROS_WARN_THROTTLE(1.0, "System Emergency!");
    filtered_commands.control = emergency_commands_.control;
    filtered_commands.shift = emergency_commands_.shift;  // tmp
  }

  // Check engage
  if (!is_engaged_) {
    filtered_commands.control.control = createStopControlCmd();
  }

  // Apply limit filtering
  filtered_commands.control.control = filterControlCommand(filtered_commands.control.control);

  // tmp: Create VehicleCmd
  autoware_vehicle_msgs::VehicleCommand vehicle_cmd;
  vehicle_cmd.header = filtered_commands.control.header;
  vehicle_cmd.control = filtered_commands.control.control;
  vehicle_cmd.shift = filtered_commands.shift.shift;
  vehicle_cmd.emergency = (use_emergency_handling_ && is_system_emergency_);

  // Add frame_id to prevent RViz warnings
  fillFrameId(&vehicle_cmd.header.frame_id, "base_link");
  fillFrameId(&filtered_commands.control.header.frame_id, "base_link");

  // Publish commands
  vehicle_cmd_pub_.publish(vehicle_cmd);
  control_cmd_pub_.publish(filtered_commands.control);

  // Save ControlCmd to steering angle when disengaged
  prev_control_cmd_ = filtered_commands.control.control;
}

void VehicleCmdGate::publishEmergencyStopControlCommands()
{
  const auto stamp = ros::Time::now();

  // ControlCommand
  autoware_control_msgs::ControlCommandStamped control_cmd;
  control_cmd.header.stamp = stamp;
  control_cmd.header.frame_id = "base_link";
  control_cmd.control = createEmergencyStopControlCmd();

  // Shift
  autoware_vehicle_msgs::ShiftStamped shift;
  shift.header.stamp = stamp;
  shift.header.frame_id = "base_link";
  shift.shift.data = autoware_vehicle_msgs::Shift::NONE;

  // TurnSignal
  autoware_vehicle_msgs::TurnSignal turn_signal;
  turn_signal.header.stamp = stamp;
  turn_signal.header.frame_id = "base_link";
  turn_signal.data = autoware_vehicle_msgs::TurnSignal::HAZARD;

  autoware_vehicle_msgs::VehicleCommand vehicle_cmd;
  vehicle_cmd.header.stamp = stamp;
  vehicle_cmd.header.frame_id = "base_link";
  vehicle_cmd.control = control_cmd.control;
  vehicle_cmd.shift = shift.shift;
  vehicle_cmd.emergency = true;

  vehicle_cmd_pub_.publish(vehicle_cmd);
  control_cmd_pub_.publish(control_cmd);
  gate_mode_pub_.publish(current_gate_mode_);
  turn_signal_cmd_pub_.publish(turn_signal);
  shift_cmd_pub_.publish(shift);
}

autoware_control_msgs::ControlCommand VehicleCmdGate::filterControlCommand(
  const autoware_control_msgs::ControlCommand & in)
{
  autoware_control_msgs::ControlCommand out = in;
  const double dt = getDt();

  filter_.limitLongitudinalWithVel(out);
  filter_.limitLongitudinalWithAcc(dt, out);
  filter_.limitLongitudinalWithJerk(dt, out);
  filter_.limitLateralWithLatAcc(dt, out);
  filter_.limitLateralWithLatJerk(dt, out);
  filter_.setPrevCmd(out);

  return out;
}

autoware_control_msgs::ControlCommand VehicleCmdGate::createStopControlCmd() const
{
  autoware_control_msgs::ControlCommand cmd;

  cmd.steering_angle = current_steer_;
  cmd.steering_angle_velocity = 0.0;
  cmd.velocity = 0.0;
  cmd.acceleration = -1.5;

  return cmd;
}

autoware_control_msgs::ControlCommand VehicleCmdGate::createEmergencyStopControlCmd() const
{
  autoware_control_msgs::ControlCommand cmd;

  cmd.steering_angle = prev_control_cmd_.steering_angle;
  cmd.steering_angle_velocity = prev_control_cmd_.steering_angle_velocity;
  cmd.velocity = 0.0;
  cmd.acceleration = -2.5;

  return cmd;
}

void VehicleCmdGate::onEngage(const std_msgs::Bool::ConstPtr msg) { is_engaged_ = msg->data; }

void VehicleCmdGate::onSystemEmergency(const std_msgs::Bool::ConstPtr msg)
{
  is_system_emergency_ = msg->data;
  system_emergency_heartbeat_received_time_ = std::make_shared<ros::Time>(ros::Time::now());
}

void VehicleCmdGate::onExternalEmergencyStop(const std_msgs::Bool::ConstPtr msg)
{
  if (msg->data) {
    is_external_emergency_stop_ = true;
  }
  external_emergency_stop_heartbeat_received_time_ = std::make_shared<ros::Time>(ros::Time::now());
}

void VehicleCmdGate::onGateMode(const autoware_control_msgs::GateMode::ConstPtr & msg)
{
  const auto prev_gate_mode = current_gate_mode_;
  current_gate_mode_ = *msg;

  if (current_gate_mode_.data != prev_gate_mode.data) {
    ROS_INFO(
      "GateMode changed: %s -> %s", getGateModeName(prev_gate_mode.data),
      getGateModeName(current_gate_mode_.data));
  }
}

void VehicleCmdGate::onSteering(const autoware_vehicle_msgs::Steering::ConstPtr msg)
{
  current_steer_ = msg->data;
}

double VehicleCmdGate::getDt()
{
  if (!prev_time_) {
    prev_time_ = std::make_shared<ros::Time>(ros::Time::now());
    return 0.0;
  }

  const auto current_time = ros::Time::now();
  const auto dt = (current_time - *prev_time_).toSec();
  *prev_time_ = current_time;

  return dt;
}

bool VehicleCmdGate::onExternalEmergencyStopService(
  std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res)
{
  is_external_emergency_stop_ = true;
  res.success = true;
  res.message = "external_emergency_stop requested was accepted.";

  return true;
}

bool VehicleCmdGate::onClearExternalEmergencyStopService(
  std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res)
{
  if (is_external_emergency_stop_) {
    if (!is_external_emergency_stop_heartbeat_timeout_) {
      is_external_emergency_stop_ = false;
      res.success = true;
      res.message = "external_emergency_stop state was cleared.";
    } else {
      res.success = false;
      res.message = "Couldn't clear external_emergency_stop state because heartbeat is timeout.";
    }
  } else {
    res.success = false;
    res.message = "Not in external_emergency_stop state.";
  }

  return true;
}

void VehicleCmdGate::checkExternalEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::DiagnosticStatus;

  DiagnosticStatus status;
  if (is_external_emergency_stop_heartbeat_timeout_) {
    status.level = DiagnosticStatus::ERROR;
    status.message = "external_emergency_stop heartbeat is timeout.";
  } else if (is_external_emergency_stop_) {
    status.level = DiagnosticStatus::ERROR;
    status.message =
      "external_emergency_stop is required. Please call `clear_external_emergency_stop` service to "
      "clear state.";
  } else {
    status.level = DiagnosticStatus::OK;
  }

  stat.summary(status.level, status.message);
}
