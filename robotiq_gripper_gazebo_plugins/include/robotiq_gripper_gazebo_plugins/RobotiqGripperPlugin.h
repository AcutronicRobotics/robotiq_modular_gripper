/*
 * Copyright 2014 Open Source Robotics Foundation
 * Copyright 2015 Clearpath Robotics
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
 *
*/

#ifndef GAZEBO_ROBOTIQ_GRIPPER_PLUGIN_HH
#define GAZEBO_ROBOTIQ_GRIPPER_PLUGIN_HH

#include <string>
#include <vector>

// Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

// HRIM
#include <hrim_actuator_gripper_msgs/msg/state_finger_gripper.hpp>
#include <hrim_actuator_gripper_srvs/srv/control_finger.hpp>

#include <boost/bind.hpp>

using namespace std::chrono_literals;

namespace gazebo
  {
  class RobotiqGripperPlugin : public gazebo::ModelPlugin
  {
    private:
      sdf::ElementPtr sdf;
      gazebo::physics::ModelPtr model;
      gazebo_ros::Node::SharedPtr node;
      gazebo::event::ConnectionPtr updateConnection;

      std::shared_ptr<rclcpp::Publisher<hrim_actuator_gripper_msgs::msg::StateFingerGripper>> fingerstatePublisher;
      std::shared_ptr<rclcpp::TimerBase> timer_fingerstate;
      rclcpp::Service<hrim_actuator_gripper_srvs::srv::ControlFinger>::SharedPtr fingercontrolService;

      std::vector<physics::JointPtr> jointsVec;
      std::map<std::string, double> joint_multipliers_;

      double target;
      double kp = 400.0;
      double ki = 1;
      double kd = 0.01;
      double imin = 0.0;
      double imax = 0.0;
      double cmdmin = -10.0;
      double cmdmax = 10.0;

      void createTopicAndService(std::string node_name);
      void gripper_service(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Request> request, std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Response> response);
      void timer_fingerstate_msgs();
      void UpdatePIDControl();
      void UpdateJointPIDs();

    public:
      RobotiqGripperPlugin();
      ~RobotiqGripperPlugin();

      void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);
  };
}
#endif
