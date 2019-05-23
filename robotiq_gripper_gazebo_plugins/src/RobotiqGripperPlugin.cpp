#include <string>
#include <vector>

#include "robotiq_gripper_gazebo_plugins/RobotiqGripperPlugin.h"

namespace gazebo{
  RobotiqGripperPlugin::RobotiqGripperPlugin(){
    printf("RobotiqGripperPlugin\n");
  }

  RobotiqGripperPlugin::~RobotiqGripperPlugin(){
  }

  void RobotiqGripperPlugin::gripper_service(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Request> request, std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Response> response){
    (void)request_header;

    if(this->model->GetName() == "hande"){
      target = request->goal_linearposition;
    }
    else
      target = request->goal_angularposition;

    response->goal_accepted = true;
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
    else
      target = jointsVec[0]->UpperLimit(0);

    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin( boost::bind(&RobotiqGripperPlugin::UpdatePIDControl, this));

    UpdateJointPIDs();
  }

  void RobotiqGripperPlugin::createTopicAndService(std::string node_name){

    std::string fingerstate = std::string(node_name) + "/fingerstate";
    RCLCPP_INFO(node->get_logger(), "creating %s publisher ", fingerstate.c_str());
    fingerstatePublisher = node->create_publisher<hrim_actuator_gripper_msgs::msg::StateFingerGripper>(fingerstate, rmw_qos_profile_default);

    timer_fingerstate = node->create_wall_timer(100ms, std::bind(&RobotiqGripperPlugin::timer_fingerstate_msgs, this));

    std::string fingercontrol = std::string(node_name) + "/fingercontrol";
    RCUTILS_LOG_INFO_NAMED(node->get_name(), "creating %s service ", fingercontrol.c_str());
    std::function<void( std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Request>, std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Response>)> cb_fingercontrol_function = std::bind( &RobotiqGripperPlugin::gripper_service, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);
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
      this->model->GetJointController()->SetPositionPID(joint->GetScopedName(), common::PID(kp, ki, kd, imax, imin, joint->LowerLimit(0), joint->UpperLimit(0)));
  }

  void RobotiqGripperPlugin::UpdatePIDControl(){
    for(auto &joint : this->jointsVec)
      this->model->GetJointController()->SetPositionTarget(joint->GetScopedName(), (joint->UpperLimit(0) - target) * joint_multipliers_[joint->GetScopedName()]);
  }

  GZ_REGISTER_MODEL_PLUGIN(RobotiqGripperPlugin)
}
