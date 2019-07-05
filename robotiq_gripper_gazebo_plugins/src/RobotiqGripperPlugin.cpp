#include "robotiq_gripper_gazebo_plugins/RobotiqGripperPlugin.h"

namespace gazebo_plugins{
  RobotiqGripperPlugin::RobotiqGripperPlugin()
  : impl_(std::make_unique<RobotiqGripperPluginPrivate>())
  {
    printf("RobotiqGripperPlugin\n");
  }

  RobotiqGripperPlugin::~RobotiqGripperPlugin(){
  }

  void RobotiqGripperPluginPrivate::gripper_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Request> request,
    std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Response> response)
  {
    if(!executing_joints){
      (void)request_header;

      // Pose control
      double upper_limit = jointsVec[0]->UpperLimit(0);

      if((int)joint_type == 1088){ // prismatic
        target_pose = request->goal_linearposition;
      }
      else if((int)joint_type == 576){ // revolute
        target_pose = request->goal_angularposition;
      }
      else{
        RCLCPP_ERROR(ros_node->get_logger(), "joint_finger type not supported");
      }

      if(target_pose < 0.0){
        target_pose = 0.0;
        RCLCPP_INFO(ros_node->get_logger(), "goal changed to its minimum value: 0.0");
      }else if(target_pose > upper_limit){
        target_pose = upper_limit;
        RCLCPP_INFO(ros_node->get_logger(), "goal changed to its maximum value: %lf", upper_limit);
      }

      // Invert pose meaning
      target_pose = upper_limit - target_pose;

      // Speed control
      if((int)joint_type == 1088){ // prismatic (e.g. S50 gripper)
        if (request->goal_velocity >= MinVelocity && request->goal_velocity <= MaxVelocity){
          target_velocity = request->goal_velocity;
        }else if (request->goal_velocity < MinVelocity){
          target_velocity = MinVelocity;
          RCLCPP_INFO(ros_node->get_logger(), "Minimum value exceeded, target velocity changed to its minimum value: %lf mm/s", MinVelocity);
        }else if (request->goal_velocity > MaxVelocity){
          target_velocity = MaxVelocity;
          RCLCPP_INFO(ros_node->get_logger(), "maximum value exceeded, target velocity changed to its maximum value: %lf mm/s", MaxVelocity);
        }
        target_velocity *= 0.001; // to m/s
      }
      else if((int)joint_type == 576){ // revolute (e.g. S140 and S85 grippers)
        if (request->goal_velocity >= MinVelocity && request->goal_velocity <= MaxVelocity){
          target_velocity = request->goal_velocity / radius;
        }else if (request->goal_velocity < MinVelocity){
          target_velocity = MinVelocity / radius;
          RCLCPP_INFO(ros_node->get_logger(), "Minimum value exceeded, target velocity changed to its minimum value: %lf mm/s", MinVelocity);
        }else if (request->goal_velocity > MaxVelocity){
          target_velocity = MaxVelocity / radius;
          RCLCPP_INFO(ros_node->get_logger(), "maximum value exceeded, target velocity changed to its maximum value: %lf mm/s", MaxVelocity);
        }
      }

      double current_pose = model->GetJointController()->GetPositions().begin()->second; //Taking first joint for reference only, this should be improved
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

  void RobotiqGripperPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf){
    impl_->model = _model;
    impl_->sdf = _sdf;

    if (!impl_->model){
      gzerr<< "Parent model is NULL! RobotiqGripperPlugin could not be loaded."<< std::endl;
      return;
    }

    std::string robot_namespace_ = "";
    if(impl_->sdf->HasElement("robotNamespace"))
       robot_namespace_ = impl_->sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    else
      printf("No robotNamespace\n");

    impl_->ros_node = gazebo_ros::Node::Get(impl_->sdf);
    std::string node_name = impl_->sdf->Get<std::string>("name");
    RCLCPP_INFO(impl_->ros_node->get_logger(), "name of the gripper node is %s\n", node_name.c_str());

    if(impl_->sdf->HasElement("kp"))
      impl_->kp = impl_->sdf->GetElement("kp")->Get<double>();
    if(impl_->sdf->HasElement("ki"))
      impl_->ki = impl_->sdf->GetElement("ki")->Get<double>();
    if(impl_->sdf->HasElement("kd"))
      impl_->kd = impl_->sdf->GetElement("kd")->Get<double>();

    // velocity related tags
    if(impl_->sdf->HasElement("min_velocity")){
      impl_->MinVelocity = impl_->sdf->GetElement("min_velocity")->Get<double>();
    }else{
      RCLCPP_ERROR(impl_->ros_node->get_logger(), "No min_velocity element.");
      impl_->ros_node.reset();
      return;
    }
    if(impl_->sdf->HasElement("max_velocity")){
      impl_->MaxVelocity = impl_->sdf->GetElement("max_velocity")->Get<double>();
    }else{
      RCLCPP_ERROR(impl_->ros_node->get_logger(), "No max_velocity element.");
      impl_->ros_node.reset();
      return;
    }
    if(impl_->sdf->HasElement("radius")){
      impl_->radius = impl_->sdf->GetElement("radius")->Get<double>();
    }else{
      RCLCPP_INFO(impl_->ros_node->get_logger(), "No radius element found. Angular movement radius must be set for grippers with revolute joints.");
    }

    sdf::ElementPtr joint_elem = impl_->sdf->GetElement("joint");
    while(joint_elem){
      auto joint_name = joint_elem->Get<std::string>();
      auto joint = impl_->model->GetJoint(joint_name);

      if(joint_name.find("joint_finger")){
        impl_->joint_type = impl_->model->GetJoint(joint_name)->GetType();
      }

      if(!joint){
        gzthrow("Could not find " + joint_name + " joint\n");
      }
      else{
        if(joint_elem->HasAttribute("multiplier"))
          impl_->joint_multipliers_[joint->GetScopedName()] = std::stod(joint_elem->GetAttribute("multiplier")->GetAsString());
        else
          impl_->joint_multipliers_[joint->GetScopedName()] = 1;
        impl_->jointsVec.push_back(joint);
      }
      joint_elem = joint_elem->GetNextElement("joint");
    }

    if((int)impl_->joint_type == -1){
      RCLCPP_ERROR(impl_->ros_node->get_logger(), "joint_finger is missing, the gripper will not work");
    }

    if(impl_->jointsVec.empty()){
      RCLCPP_ERROR(impl_->ros_node->get_logger(), "No joints found.");
      impl_->ros_node.reset();
      return;
    }
    else{
      impl_->targetJoint = 0;
    }

    impl_->interpolated_targetJoint.clear();
    impl_->executing_joints = false;
    impl_->index_executing_joints = 0;

    // Create ROS2 gripper state publisher with timer + Finger Control subscription service.
    createTopicAndService(node_name);

    // Listen to the update event (broadcast every simulation iteration)
    impl_->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RobotiqGripperPluginPrivate::UpdatePIDControl, impl_.get()));

    impl_->UpdateJointPIDs();

    std::cout.flush();
  }

  void RobotiqGripperPlugin::Reset()
  {
    impl_->interpolated_targetJoint.clear();
    impl_->targetJoint = 0;
    impl_->executing_joints = false;
  }

  void RobotiqGripperPlugin::createTopicAndService(std::string node_name){

    std::string fingerstate = std::string(node_name) + "/fingerstate";
    RCLCPP_INFO(impl_->ros_node->get_logger(), "creating %s publisher ", fingerstate.c_str());
    impl_->fingerstatePublisher = impl_->ros_node->create_publisher<hrim_actuator_gripper_msgs::msg::StateFingerGripper>(
      fingerstate, rclcpp::QoS(1));

    impl_->timer_fingerstate = impl_->ros_node->create_wall_timer(
      1s, std::bind(&RobotiqGripperPluginPrivate::timer_fingerstate_msgs, impl_.get()));

    std::string fingercontrol = std::string(node_name) + "/fingercontrol";
    RCUTILS_LOG_INFO_NAMED(impl_->ros_node->get_name(), "creating %s service ", fingercontrol.c_str());
    std::function<void(std::shared_ptr<rmw_request_id_t>,
                       const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Request>,
                       std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Response>)> cb_fingercontrol_function = std::bind(
                       &RobotiqGripperPluginPrivate::gripper_service, impl_.get(), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    impl_->fingercontrolService = impl_->ros_node->create_service<hrim_actuator_gripper_srvs::srv::ControlFinger>(fingercontrol, cb_fingercontrol_function);
  }

  void RobotiqGripperPluginPrivate::timer_fingerstate_msgs(){

    std::cout << "TIMER MSG STATE 0" << std::endl;

    hrim_actuator_gripper_msgs::msg::StateFingerGripper fingerstateMsg;
    gazebo::common::Time cur_time = model->GetWorld()->SimTime();
    fingerstateMsg.header.stamp.sec = cur_time.sec;
    fingerstateMsg.header.stamp.nanosec = cur_time.nsec;

    if((int)joint_type == 1088){
      // prismatic
      fingerstateMsg.linear_position = jointsVec.front()->Position(0);
      fingerstateMsg.angular_position = 0;
    }
    else if((int)joint_type == 576){
      // revolute
      fingerstateMsg.linear_position = 0;
      fingerstateMsg.angular_position = jointsVec.front()->Position(0);
    }else{
      std::cout << "WHAT???" << std::endl;

    }

    std::cout << "TIMER MSG STATE" << std::endl;
    std::cout << fingerstateMsg.header.stamp.sec << std::endl;
    std::cout << fingerstateMsg.header.stamp.nanosec << std::endl;
    std::cout << fingerstateMsg.linear_position << std::endl;
    std::cout << fingerstateMsg.angular_position << std::endl;

    fingerstatePublisher->publish(fingerstateMsg);
    std::cout << "PUBLISHED" << std::endl;
    std::cout.flush();
  }

  void RobotiqGripperPluginPrivate::UpdateJointPIDs(){
    for(auto &joint : jointsVec)
      model->GetJointController()->SetPositionPID(joint->GetScopedName(),
        gazebo::common::PID(kp, ki, kd, imax, imin, joint->LowerLimit(0), joint->UpperLimit(0)));
  }

  void RobotiqGripperPluginPrivate::UpdatePIDControl(){
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
    for(auto &joint : jointsVec)
      model->GetJointController()->SetPositionTarget(joint->GetScopedName(),
        targetJoint * joint_multipliers_[joint->GetScopedName()]);
  }

  GZ_REGISTER_MODEL_PLUGIN(RobotiqGripperPlugin)
}
