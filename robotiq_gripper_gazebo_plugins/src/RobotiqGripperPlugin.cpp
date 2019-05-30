#include <string>
#include <vector>

#include "robotiq_gripper_gazebo_plugins/RobotiqGripperPlugin.h"

namespace gazebo{
  RobotiqGripperPlugin::RobotiqGripperPlugin(){
    printf("RobotiqGripperPlugin\n");
  }

  RobotiqGripperPlugin::~RobotiqGripperPlugin(){
  }

  void RobotiqGripperPlugin::gripper_service(const std::shared_ptr<rmw_request_id_t> request_header,
                                             const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Request> request,
                                             std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Response> response){
    if(!executing_joints){
      (void)request_header;

      // Pose control
      double upper_limit = jointsVec[0]->UpperLimit(0);

      if(this->model->GetName() == "hande"){
        target_pose = request->goal_linearposition;
      }else if(this->model->GetName() == "robotiq_85" || this->model->GetName() == "robotiq_140"){
        target_pose = request->goal_angularposition;
      }

      if(target_pose < 0.0){
        target_pose = 0.0;
        RCLCPP_INFO(node->get_logger(), "goal changed to its minimum value: 0.0");
      }else if(target_pose > upper_limit){
        target_pose = upper_limit;
        RCLCPP_INFO(node->get_logger(), "goal changed to its maximum value: %lf", upper_limit);
      }

      // Invert pose meaning
      target_pose = upper_limit - target_pose;

      // Speed control
      if(gripper_id == 50){
        if (request->goal_velocity >= MinVelocity_s50 && request->goal_velocity <= MaxVelocity_s50){
          target_velocity = request->goal_velocity;
        }else if (request->goal_velocity < MinVelocity_s50){
          target_velocity = MinVelocity_s50;
          RCLCPP_INFO(node->get_logger(), "Minimum value exceeded, target velocity changed to its minimum value: %lf mm/s", MinVelocity_s50);
        }else if (request->goal_velocity > MaxVelocity_s50){
          target_velocity = MaxVelocity_s50;
          RCLCPP_INFO(node->get_logger(), "maximum value exceeded, target velocity changed to its maximum value: %lf mm/s", MaxVelocity_s50);
        }
        target_velocity *= 0.001; // to m/s
      }else if(gripper_id == 85){
        if (request->goal_velocity >= MinVelocity_s85 && request->goal_velocity <= MaxVelocity_s85){
          target_velocity = request->goal_velocity / radius_s85;
        }else if (request->goal_velocity < MinVelocity_s85){
          target_velocity = MinVelocity_s85 / radius_s85;
          RCLCPP_INFO(node->get_logger(), "Minimum value exceeded, target velocity changed to its minimum value: %lf mm/s", MinVelocity_s85);
        }else if (request->goal_velocity > MaxVelocity_s85){
          target_velocity = MaxVelocity_s85 / radius_s85;
          RCLCPP_INFO(node->get_logger(), "maximum value exceeded, target velocity changed to its maximum value: %lf mm/s", MaxVelocity_s85);
        }
      }else if(gripper_id == 140){
        if (request->goal_velocity >= MinVelocity_s140 && request->goal_velocity <= MaxVelocity_s140){
          target_velocity = request->goal_velocity / radius_s140;
        }else if (request->goal_velocity < MinVelocity_s140){
          target_velocity = MinVelocity_s140 / radius_s140;
          RCLCPP_INFO(node->get_logger(), "Minimum value exceeded, target velocity changed to its minimum value: %lf mm/s", MinVelocity_s140);
        }else if (request->goal_velocity > MaxVelocity_s140){
          target_velocity = MaxVelocity_s140 / radius_s140;
          RCLCPP_INFO(node->get_logger(), "maximum value exceeded, target velocity changed to its maximum value: %lf mm/s", MaxVelocity_s140);
        }
      }

      double current_pose = this->model->GetJointController()->GetPositions().begin()->second; //Taking first joint for reference only, this should be improved
      double start_time = 0;
      // ATTENTION! Same formula is used for both angular and linear movement grippers. Meaning the values in the formula sometimes contain radians, other times meters.
      double end_time = fabs(current_pose - target_pose) / target_velocity;

      std::vector<double> X(2), Y_pos(2);
      X[0] = start_time;
      X[1] = end_time;
      Y_pos[0] = current_pose;
      Y_pos[1] = target_pose;

      tk::spline interpolation_linear_pos;
      if(!interpolation_linear_pos.set_points(X, Y_pos))
        return;

      interpolated_targetJoint.clear();
      for(double t = start_time; t < end_time; t+=0.001 ){
        interpolated_targetJoint.push_back(interpolation_linear_pos(t));
      }

      response->goal_accepted = true;
    }
  }

  void RobotiqGripperPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf){
    this->model = parent;
    this->sdf = sdf;

    if (!this->model){
      gzerr<< "Parent model is NULL! RobotiqGripperPlugin could not be loaded."<< std::endl;
      return;
    }

    std::string robot_namespace_ = "";
    if(this->sdf->HasElement("robotNamespace"))
       robot_namespace_ = this->sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    else
      printf("No robotNamespace\n");

    node = gazebo_ros::Node::Get(this->sdf);
    std::string node_name = this->sdf->Get<std::string>("name");
    RCLCPP_INFO(node->get_logger(), "name of the gripper node is %s\n", node_name.c_str());

    createTopicAndService(node_name);

    if(this->sdf->HasElement("id")){
       this->gripper_id = this->sdf->GetElement("id")->Get<int>();
    }else{
      RCLCPP_ERROR(node->get_logger(), "No id element.");
      node.reset();
      return;
    }

    if(this->sdf->HasElement("kp"))
      this->kp = this->sdf->GetElement("kp")->Get<double>();
    if(this->sdf->HasElement("ki"))
      this->ki = this->sdf->GetElement("ki")->Get<double>();
    if(this->sdf->HasElement("kd"))
      this->kd = this->sdf->GetElement("kd")->Get<double>();

    sdf::ElementPtr joint_elem = this->sdf->GetElement("joint");
    while(joint_elem){
      auto joint_name = joint_elem->Get<std::string>();
      auto joint = model->GetJoint(joint_name);

      if(!joint){
        gzthrow("Could not find " + joint_name + " joint\n");
      }
      else{
        if(joint_elem->HasAttribute("multiplier"))
          joint_multipliers_[joint->GetScopedName()] = std::stod(joint_elem->GetAttribute("multiplier")->GetAsString());
        else
          joint_multipliers_[joint->GetScopedName()] = 1;
        jointsVec.push_back(joint);
      }
      joint_elem = joint_elem->GetNextElement("joint");
    }

    if(jointsVec.empty()){
      RCLCPP_ERROR(node->get_logger(), "No joints found.");
      node.reset();
      return;
    }
    else{
      targetJoint = 0;
    }

    interpolated_targetJoint.clear();
    executing_joints = false;
    index_executing_joints = 0;

    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin( boost::bind(&RobotiqGripperPlugin::UpdatePIDControl, this));

    UpdateJointPIDs();
  }

  void RobotiqGripperPlugin::Reset()
  {
    interpolated_targetJoint.clear();
    targetJoint = 0;
    executing_joints = false;
  }

  void RobotiqGripperPlugin::createTopicAndService(std::string node_name){

    std::string fingerstate = std::string(node_name) + "/fingerstate";
    RCLCPP_INFO(node->get_logger(), "creating %s publisher ", fingerstate.c_str());
    fingerstatePublisher = node->create_publisher<hrim_actuator_gripper_msgs::msg::StateFingerGripper>(fingerstate, rmw_qos_profile_default);

    timer_fingerstate = node->create_wall_timer(100ms, std::bind(&RobotiqGripperPlugin::timer_fingerstate_msgs, this));

    std::string fingercontrol = std::string(node_name) + "/fingercontrol";
    RCUTILS_LOG_INFO_NAMED(node->get_name(), "creating %s service ", fingercontrol.c_str());
    std::function<void(std::shared_ptr<rmw_request_id_t>,
                       const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Request>,
                       std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Response>)> cb_fingercontrol_function = \
                       std::bind(&RobotiqGripperPlugin::gripper_service, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    fingercontrolService = node->create_service<hrim_actuator_gripper_srvs::srv::ControlFinger>(fingercontrol, cb_fingercontrol_function);
  }

  void RobotiqGripperPlugin::timer_fingerstate_msgs(){
    hrim_actuator_gripper_msgs::msg::StateFingerGripper fingerstateMsg;
    gazebo::common::Time cur_time = this->model->GetWorld()->SimTime();
    fingerstateMsg.header.stamp.sec = cur_time.sec;
    fingerstateMsg.header.stamp.nanosec = cur_time.nsec;

    if(this->model->GetName() == "hande"){
      fingerstateMsg.linear_position = jointsVec.front()->Position(0);
      fingerstateMsg.angular_position = 0;
    }
    else{
      fingerstateMsg.linear_position = 0;
      fingerstateMsg.angular_position = jointsVec.front()->Position(0);
    }
    fingerstatePublisher->publish(fingerstateMsg);
  }

  void RobotiqGripperPlugin::UpdateJointPIDs(){
    for(auto &joint : this->jointsVec)
      this->model->GetJointController()->SetPositionPID(joint->GetScopedName(),
        common::PID(kp, ki, kd, imax, imin, joint->LowerLimit(0), joint->UpperLimit(0)));
  }

  void RobotiqGripperPlugin::UpdatePIDControl(){
    if(!executing_joints && interpolated_targetJoint.size()>0){
      index_executing_joints = 0;
      executing_joints = true;
    }
    if(executing_joints){
      targetJoint = interpolated_targetJoint[index_executing_joints];
      index_executing_joints++;
      if(index_executing_joints==interpolated_targetJoint.size()){
        executing_joints = false;
        interpolated_targetJoint.clear();
        index_executing_joints = 0;
      }
    }
    for(auto &joint : this->jointsVec)
      this->model->GetJointController()->SetPositionTarget(joint->GetScopedName(),
        targetJoint * joint_multipliers_[joint->GetScopedName()]);
  }

  GZ_REGISTER_MODEL_PLUGIN(RobotiqGripperPlugin)
}
