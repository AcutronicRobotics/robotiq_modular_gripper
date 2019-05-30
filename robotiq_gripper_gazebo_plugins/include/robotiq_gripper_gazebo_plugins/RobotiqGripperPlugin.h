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

// Spline (interpolation)
#include <robotiq_gripper_gazebo_plugins/spline.hpp>

// HRIM
#include <hrim_actuator_gripper_msgs/msg/state_finger_gripper.hpp>
#include <hrim_actuator_gripper_srvs/srv/control_finger.hpp>

#include <boost/bind.hpp>

#include "robotiq_gripper_gazebo_plugins/GazeboGraspGripper.h"

using namespace std::chrono_literals;

// Working with gazebo >8
namespace gz_math = ignition::math;
typedef  ignition::math::Vector3d gz_math_Vector3d;
#define gz_math_Pose3d ignition::math::Pose3d
#define gz_math_Matrix4d ignition::math::Matrix4d
#define gz_math_Matrix3d ignition::math::Matrix3d



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

      double target_pose;
      double target_velocity;
      double kp = 400.0;
      double ki = 1;
      double kd = 0.01;
      double imin = 0.0;
      double imax = 0.0;
      double cmdmin = -10.0;
      double cmdmax = 10.0;

      // S140 values. Min and max joint speed values:  30 to 250 mm/s
      const double MinVelocity_s140 = 30; //1.6;
      const double MaxVelocity_s140 = 250; //0.19;
      const double radius_s140 = 156; //mm
      // S85 values. Min and max joint speed values:  20 to 150 mm/s
      const double MinVelocity_s85 = 20; //1.5;
      const double MaxVelocity_s85 = 150;// 0.2;
      const double radius_s85 = 100; //mm
      // S50 values. Min and max joint speed values:  20 to 150 mm/s
      const double MinVelocity_s50 = 20;
      const double MaxVelocity_s50 = 150;


      void createTopicAndService(std::string node_name);
      void gripper_service(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Request> request, std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Response> response);
      void timer_fingerstate_msgs();
      void OnUpdate();
      void UpdateJointPIDs();

      //  ####### grip-fix plugin related #######
      virtual void Init();
      void InitValues();

      /**
       * Gets called upon detection of contacts.
       * A list of contacts is passed in \_msg. One contact has two bodies, and only
       * the ones where one of the bodies is a gripper link are considered.
       * Each contact consists of a *list* of forces with their own origin/position each
       * (e.g. when the object and gripper are colliding at several places).
       * The averages of each contact's force vectors along with their origins is computed.
       * This "average contact force/origin" for each contact is then added to the \e this->contacts map.
       * If an entry for this object/link pair already exists, the average force (and its origin)
       * is *added* to the existing force/origin, and the average count is increased. This is to get
       * the average force application over time between link and object.
       */
      void OnContact(const ConstContactsPtr &ptr);

      //    bool CheckGrip(const std::vector<GzVector3> &forces, float minAngleDiff,
      //                   float lengthRatio);

      bool IsGripperLink(const std::string &linkName, std::string &gripperName) const;

      /**
       * return objects (key) and the gripper (value) to which it is attached
       */
      std::map<std::string, std::string> GetAttachedObjects() const;

      /**
       * Helper class to collect contact info per object.
       * Forward declaration here.
       */
      class ObjectContactInfo;

      /**
       * Helper function to determine if object attached to a gripper in ObjectContactInfo.
       */
      bool ObjectAttachedToGripper(const ObjectContactInfo &objContInfo,
                                   std::string &attachedToGripper) const;

      /**
       * Helper function to determine if object attached to this gripper
       */
      bool ObjectAttachedToGripper(const std::string &gripperName,
                                   std::string &attachedToGripper) const;


      // physics::ModelPtr model;
      // physics::PhysicsEnginePtr physics;
      physics::WorldPtr world;

      // sorted by their name, all grippers of the robot
      std::map<std::string, GazeboGraspGripper> grippers;

      event::ConnectionPtr update_connection;
      transport::NodePtr transp_node;
      transport::SubscriberPtr contactSub; //subscriber to contact updates

      // tolerance (in degrees) between force vectors to
      // beconsidered "opposing"
      float forcesAngleTolerance;

      // when an object is attached, collisions with it may be disabled, in case the
      // robot still keeps wobbling.
      bool disableCollisionsOnAttach;

      // all collisions per gazebo collision link (each entry
      // belongs to a physics::CollisionPtr element). The key
      // is the collision link name, the value is the gripper name
      // this collision link belongs to.
      std::map<std::string, std::string> collisions;


      /**
       * Helper class to encapsulate a collision information. Forward declaration here.
       */
      class CollidingPoint;

      // Contact forces sorted by object name the gripper collides with (first key)
      // and the link colliding (second key).
      std::map<std::string, std::map<std::string, CollidingPoint> > contacts;
      boost::mutex mutexContacts; //mutex protects contacts

      // when an object was first attached, it had these colliding points.
      // First key is object name, second is the link colliding, as in \e contacts.
      // Only the links of *one* gripper are stored here. This indirectly imposes the
      // limitation that no two grippers can grasp the object (while it would be
      // possible, the release condition is tied to only one link, so the object may
      // not be released properly).
      std::map<std::string, std::map<std::string, CollidingPoint> >
      attachGripContacts;


      // Records how many subsequent update calls the grip on that object has been recorded
      // as "holding". Every loop, if a grip is not recorded, this number decreases.
      // When it reaches \e grip_count_threshold, it will be attached.
      // The number won't increase above max_grip_count once it has reached that number.
      std::map<std::string, int> gripCounts;

      // *maximum* number in \e gripCounts to be recorded.
      int maxGripCount;

      // number of recorded "grips" in the past (in gripCount) which, when it is exceeded, counts
      // as the object grasped, and when it is lower, as released.
      int gripCountThreshold;

      // once an object is gripped, the relative position of the collision link surface to the
      // object is remembered. As soon as this distance changes more than release_tolerance,
      // the object is released.
      float releaseTolerance;

      //nano seconds between two updates
      common::Time updateRate;

      //last time OnUpdate() was called
      common::Time prevUpdateTime;

    public:
      RobotiqGripperPlugin();
      ~RobotiqGripperPlugin();

      void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);
      void Reset();

      // Interpolation
      std::vector<float> interpolated_targetJoint;
      double current_pose_rad = 0.0;
      bool executing_joints;
      unsigned int index_executing_joints;
      double targetJoint  = 0.0;

      //  ####### grip-fix plugin related #######
      /**
      * Gets called just after the object has been attached to the palm link on \e armName
      */
      virtual void OnAttach(const std::string &objectName,
                            const std::string &armName) {}
      /**
       * Gets called just after the object has been detached to the palm link on \e armName
       */
      virtual void OnDetach(const std::string &objectName,
                            const std::string &armName) {}


  };
}
#endif  // GAZEBO_ROBOTIQ_GRIPPER_PLUGIN_HH
