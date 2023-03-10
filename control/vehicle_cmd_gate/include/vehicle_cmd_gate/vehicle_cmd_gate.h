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

#ifndef VEHICLE_CMD_GATE_VEHICLE_CMD_GATE_H
#define VEHICLE_CMD_GATE_VEHICLE_CMD_GATE_H

#include <memory>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_control_msgs/GateMode.h>
#include <autoware_vehicle_msgs/ShiftStamped.h>
#include <autoware_vehicle_msgs/Steering.h>
#include <autoware_vehicle_msgs/TurnSignal.h>
#include <autoware_vehicle_msgs/VehicleCommand.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include "vehicle_cmd_gate/vehicle_cmd_filter.h"

struct Commands
{
  autoware_control_msgs::ControlCommandStamped control;
  autoware_vehicle_msgs::TurnSignal turn_signal;
  autoware_vehicle_msgs::ShiftStamped shift;
};

class VehicleCmdGate
{
public:
  VehicleCmdGate();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle pnh_{"~"};

  // Publisher
  ros::Publisher vehicle_cmd_pub_;
  ros::Publisher control_cmd_pub_;
  ros::Publisher shift_cmd_pub_;
  ros::Publisher turn_signal_cmd_pub_;
  ros::Publisher gate_mode_pub_;

  // Subscriber
  ros::Subscriber engage_sub_;
  ros::Subscriber system_emergency_sub_;
  ros::Subscriber external_emergency_stop_sub_;
  ros::Subscriber gate_mode_sub_;
  ros::Subscriber steer_sub_;

  void onGateMode(const autoware_control_msgs::GateMode::ConstPtr & msg);
  void onEngage(const std_msgs::Bool::ConstPtr msg);
  void onSystemEmergency(const std_msgs::Bool::ConstPtr msg);
  void onExternalEmergencyStop(const std_msgs::Bool::ConstPtr msg);
  void onSteering(const autoware_vehicle_msgs::Steering::ConstPtr msg);

  bool is_engaged_ = false;
  bool is_system_emergency_ = false;
  bool is_external_emergency_stop_ = false;
  double current_steer_ = 0;
  autoware_control_msgs::GateMode current_gate_mode_;

  // Heartbeat
  std::shared_ptr<ros::Time> system_emergency_heartbeat_received_time_;
  bool is_system_emergency_heartbeat_timeout_ = false;

  std::shared_ptr<ros::Time> external_emergency_stop_heartbeat_received_time_;
  bool is_external_emergency_stop_heartbeat_timeout_ = false;

  // Subscriber for auto
  Commands auto_commands_;
  ros::Subscriber auto_control_cmd_sub_;
  ros::Subscriber auto_turn_signal_cmd_sub_;
  ros::Subscriber auto_shift_cmd_sub_;
  void onAutoCtrlCmd(const autoware_control_msgs::ControlCommandStamped::ConstPtr & msg);
  void onAutoTurnSignalCmd(const autoware_vehicle_msgs::TurnSignal::ConstPtr & msg);
  void onAutoShiftCmd(const autoware_vehicle_msgs::ShiftStamped::ConstPtr & msg);

  // Subscriber for remote
  Commands remote_commands_;
  ros::Subscriber remote_control_cmd_sub_;
  ros::Subscriber remote_turn_signal_cmd_sub_;
  ros::Subscriber remote_shift_cmd_sub_;
  void onRemoteCtrlCmd(const autoware_control_msgs::ControlCommandStamped::ConstPtr & msg);
  void onRemoteTurnSignalCmd(const autoware_vehicle_msgs::TurnSignal::ConstPtr & msg);
  void onRemoteShiftCmd(const autoware_vehicle_msgs::ShiftStamped::ConstPtr & msg);

  // Subscriber for emergency
  Commands emergency_commands_;
  ros::Subscriber emergency_control_cmd_sub_;
  ros::Subscriber emergency_turn_signal_cmd_sub_;
  ros::Subscriber emergency_shift_cmd_sub_;
  void onEmergencyCtrlCmd(const autoware_control_msgs::ControlCommandStamped::ConstPtr & msg);
  void onEmergencyTurnSignalCmd(const autoware_vehicle_msgs::TurnSignal::ConstPtr & msg);
  void onEmergencyShiftCmd(const autoware_vehicle_msgs::ShiftStamped::ConstPtr & msg);

  // Parameter
  double update_rate_;
  bool use_emergency_handling_;
  bool use_external_emergency_stop_;
  double system_emergency_heartbeat_timeout_;
  double external_emergency_stop_heartbeat_timeout_;

  // Service
  ros::ServiceServer srv_external_emergency_stop_;
  ros::ServiceServer srv_clear_external_emergency_stop_;

  bool onExternalEmergencyStopService(
    std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  bool onClearExternalEmergencyStopService(
    std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);

  // Timer / Event
  ros::Timer timer_;

  void onTimer(const ros::TimerEvent & event);
  void publishControlCommands(const Commands & input_msg);
  void publishEmergencyStopControlCommands();

  // Diagnostics Updater
  diagnostic_updater::Updater updater_;

  void checkExternalEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Algorithm
  autoware_control_msgs::ControlCommand prev_control_cmd_;
  autoware_control_msgs::ControlCommand createStopControlCmd() const;
  autoware_control_msgs::ControlCommand createEmergencyStopControlCmd() const;

  std::shared_ptr<ros::Time> prev_time_;
  double getDt();

  VehicleCmdFilter filter_;
  autoware_control_msgs::ControlCommand filterControlCommand(
    const autoware_control_msgs::ControlCommand & msg);
};

#endif  // VEHICLE_CMD_GATE_VEHICLE_CMD_GATE_H
