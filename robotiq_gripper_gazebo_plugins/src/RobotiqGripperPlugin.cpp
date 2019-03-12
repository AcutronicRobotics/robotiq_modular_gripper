#include <string>
#include <vector>

#include "robotiq_gripper_gazebo_plugins/RobotiqGripperPlugin.h"

namespace gazebo
  {
  ////////////////////////////////////////////////////////////////////////////////
  RobotiqGripperPlugin::RobotiqGripperPlugin()
  {
    printf("RobotiqGripperPlugin\n");
  }

  ////////////////////////////////////////////////////////////////////////////////
  RobotiqGripperPlugin::~RobotiqGripperPlugin()
  {
    // gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  }

  ////////////////////////////////////////////////////////////////////////////////
  bool RobotiqGripperPlugin::IsHandFullyOpen()
  {
    bool fingersOpen = true;

    // The hand will be fully open when all the fingers are within 'tolerance'
    // from their lower limits.
    ignition::math::Angle tolerance;
    tolerance.Degree(1.0);

    fingersOpen = fingersOpen && (joint_v_.front()->Position(0) < (joint_v_.front()->LowerLimit(0) + tolerance.Radian()));

    return fingersOpen;
  }

  ////////////////////////////////////////////////////////////////////////////////
  uint8_t RobotiqGripperPlugin::GetCurrentPosition(
    const gazebo::physics::JointPtr &_joint)
  {
    // Full range of motion.
    ignition::math::Angle range =
      _joint->UpperLimit(0) - _joint->LowerLimit(0);

    // Angle relative to the lower limit.
    ignition::math::Angle relAngle = _joint->Position(0) - _joint->LowerLimit(0);

    return
      static_cast<uint8_t>(round(255.0 * relAngle.Radian() / range.Radian()));
  }

  void RobotiqGripperPlugin::gripper_service(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Request> request,
        std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Response> response)
  {
    (void)request_header;

    targetJoint = request->goal_linearposition;

    UpdateJointPIDs();

    auto jointPIDS = this->model->GetJointController()->GetPositionPIDs();

    gzmsg << "Position PID parameters for joints "  << std::endl
          << "\tKP: "     << jointPIDS[joint_v_.front()->GetScopedName()].GetPGain()  << std::endl
          << "\tKI: "     << jointPIDS[joint_v_.front()->GetScopedName()].GetIGain()  << std::endl
          << "\tKD: "     << jointPIDS[joint_v_.front()->GetScopedName()].GetDGain()  << std::endl
          << "\tIMin: "   << jointPIDS[joint_v_.front()->GetScopedName()].GetIMin()   << std::endl
          << "\tIMax: "   << jointPIDS[joint_v_.front()->GetScopedName()].GetIMax()   << std::endl
          << "\tCmdMin: " << jointPIDS[joint_v_.front()->GetScopedName()].GetCmdMin() << std::endl
          << "\tCmdMax: " << jointPIDS[joint_v_.front()->GetScopedName()].GetCmdMax() << std::endl
          << std::endl;

    response->goal_accepted = true;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void RobotiqGripperPlugin::Load(gazebo::physics::ModelPtr _parent,
                               sdf::ElementPtr _sdf)
  {

    printf("RobotiqGripperPlugin::Load\n");
    gzmsg << "RobotiqGripperPlugin::Load" << std::endl;
    this->model = _parent;
    this->world = this->model->GetWorld();
    this->sdf = _sdf;

    // Error message if the model couldn't be found
    if (!this->model){
      gzerr<< "Parent model is NULL! RobotiqGripperPlugin could not be loaded."<< std::endl;
      return;
    }

    std::string robot_namespace_ = "";
    if (_sdf->HasElement("robotNamespace"))
       robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    else{
      printf("No robotNamespace\n");
    }

    gzmsg<< "robot_namespace_ " << robot_namespace_ << std::endl;

    gzmsg<< "_sdf description " << _sdf->GetDescription() << std::endl;

    for(unsigned int i = 0; i < model->GetJoints().size(); i++){
        gzmsg << this->model->GetJoints()[i]->GetScopedName() << std::endl;
    }

    // Initialize ROS node
    ros_node_ = gazebo_ros::Node::Get(_sdf);

    std::string node_name = _sdf->Get<std::string>("name");
    RCLCPP_INFO(ros_node_->get_logger(), "name %s\n", node_name.c_str());

    createGenericTopics(node_name);

    if (_sdf->HasElement("kp")){
      kp = _sdf->GetElement("kp")->Get<double>();
      RCLCPP_INFO(ros_node_->get_logger(), "Taking kp value of [%s] from plugin", std::to_string(kp).c_str());
    }else{
      RCLCPP_INFO(ros_node_->get_logger(), "Using default kp value of [%s]", std::to_string(kp).c_str());
    }

    if (_sdf->HasElement("ki")){
      ki = _sdf->GetElement("ki")->Get<double>();
      RCLCPP_INFO(ros_node_->get_logger(), "Taking ki value of [%s] from plugin", std::to_string(ki).c_str());
    }else{
      RCLCPP_INFO(ros_node_->get_logger(), "Using default ki value of [%s]", std::to_string(ki).c_str());
    }

    if (_sdf->HasElement("kd")){
      kd = _sdf->GetElement("kd")->Get<double>();
      RCLCPP_INFO(ros_node_->get_logger(), "Taking kd value of [%s] from plugin", std::to_string(kd).c_str());
    }else{
      RCLCPP_INFO(ros_node_->get_logger(), "Using default kd value of [%s]", std::to_string(kd).c_str());
    }

    ros_node_->set_parameters({
      rclcpp::Parameter("kp_gripper", kp),
      rclcpp::Parameter("ki_gripper", ki),
      rclcpp::Parameter("kd_gripper", kd),
      rclcpp::Parameter("imin_gripper", imin),
      rclcpp::Parameter("imax_gripper", imax),
      rclcpp::Parameter("cmdmin_gripper", cmdmin),
      rclcpp::Parameter("cmdmax_gripper", cmdmax)
    });

    auto param_change_callback =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        rclcpp::ParameterType parameter_type = parameter.get_type();

        int error = 0;

        if(!parameter.get_name().compare("kp_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->kp = parameter.as_double();
          }else{
            error = 1;
          }
        }else if(!parameter.get_name().compare("ki_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->ki = parameter.as_double();
          }else{
            error = 1;
          }
        }else if(!parameter.get_name().compare("kd_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->kd = parameter.as_double();
          }else{
            error = 1;
          }
        }else if(!parameter.get_name().compare("imin_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->imin = parameter.as_double();
          }else{
            error = 1;
          }
        }else if(!parameter.get_name().compare("imax_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->imax = parameter.as_double();
          }else{
            error = 1;
          }
        }else if(!parameter.get_name().compare("cmdmin_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->cmdmin = parameter.as_double();
          }else{
            error = 1;
          }
        }else if(!parameter.get_name().compare("cmdmax_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->cmdmax = parameter.as_double();
          }else{
            error = 1;
          }
        }else{
          error = 2;
        }

        if(error==1){
          RCLCPP_INFO(ros_node_->get_logger(),
            "requested value for parameter '%s' is not the right type",
            parameter.get_name().c_str());
            result.successful = false;
        }
        if(error==2){
          RCLCPP_INFO(ros_node_->get_logger(), "Parameter %s doesn't exit",   parameter.get_name().c_str());
          result.successful = false;
        }
      }
    };
    ros_node_->register_param_change_callback(param_change_callback);

    sdf::ElementPtr joint_elem = sdf->GetElement("joint");
    while (joint_elem) {
      auto joint_name = joint_elem->Get<std::string>();

      auto joint = model->GetJoint(joint_name);

      if (!joint) {
        gzthrow("Could not find "+joint_name+" joint\n");
      } else {
        if (joint_elem->HasAttribute("multiplier")){
          joint_multipliers_[joint->GetScopedName()] = std::stod(joint_elem->GetAttribute("multiplier")->GetAsString());
        }else{
          joint_multipliers_[joint->GetScopedName()] = 1;
        }
        joint_v_.push_back(joint);
        RCLCPP_INFO(ros_node_->get_logger(), "Found joint [%s]", joint_name.c_str());
      }

      joint_elem = joint_elem->GetNextElement("joint");
    }

    if (joint_v_.empty()) {
      RCLCPP_ERROR(ros_node_->get_logger(), "No joints found.");
      ros_node_.reset();
      return;
    }


    std::function<void( std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Request>,
                        std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Response>)> cb_fingercontrol_function = std::bind(
          &RobotiqGripperPlugin::gripper_service, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);

    srv_ = ros_node_->create_service<hrim_actuator_gripper_srvs::srv::ControlFinger>(node_name + "/goal", cb_fingercontrol_function);

    std::string topic_name_gripper_finger_state = std::string(node_name) + "/state_finger_gripper";
    gripper_finger_state_pub = ros_node_->create_publisher<hrim_actuator_gripper_msgs::msg::StateFingerGripper>(topic_name_gripper_finger_state,
                  rmw_qos_profile_default);

    std::string topic_name_gripper_state = std::string(node_name) + "/state_gripper";
    gripper_state_pub = ros_node_->create_publisher<hrim_actuator_gripper_msgs::msg::StateGripper>(topic_name_gripper_state,
                  rmw_qos_profile_default);

    // Connect to gazebo world update.
    this->updateConnection =
      gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RobotiqGripperPlugin::UpdatePIDControl, this));

    UpdateJointPIDs();
  }

  void RobotiqGripperPlugin::UpdateJointPIDs(){
    for(auto &joint : this->joint_v_){
      this->model->GetJointController()->SetPositionPID(
        joint->GetScopedName(),
        common::PID(kp, ki, kd, imax, imin, joint->LowerLimit(0), joint->UpperLimit(0)));
    }
  }

  void RobotiqGripperPlugin::UpdatePIDControl()
  {
    // Set the joint's target velocity.
    for(auto &joint : this->joint_v_){
      this->model->GetJointController()->SetPositionTarget(
        joint->GetScopedName(), targetJoint * joint_multipliers_[joint->GetScopedName()]);
    }
  }

  void RobotiqGripperPlugin::createGenericTopics(std::string node_name)
  {
    // create info topic
    std::string service_name_id = std::string(node_name) + "/id";

    // Creating status topic name
    std::string topic_name_status = std::string(node_name) + "/status";

    // Creating power topic name
    std::string topic_name_power = std::string(node_name) + "/power";

    // Creating sim topic name
    std::string service_name_sim3d = std::string(node_name) + "/module_3d";
    std::string service_name_simurdf = std::string(node_name) + "/module_urdf";
    std::string service_name_specs = std::string(node_name) + "/specs";
    std::string service_name_specs_comm = std::string(node_name) + "/specs_comm";
    std::string topic_name_state_comm = std::string(node_name) + "/state_comm";


    std::function<void( std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<hrim_generic_srvs::srv::ID::Request>,
                        std::shared_ptr<hrim_generic_srvs::srv::ID::Response>)> cb_id_function = std::bind(
          &RobotiqGripperPlugin::IDService, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);

    id_srv_ = ros_node_->create_service<hrim_generic_srvs::srv::ID>(service_name_id, cb_id_function);
    RCUTILS_LOG_INFO_NAMED(ros_node_->get_name(), "creating service called: %s ", service_name_id.c_str());

    power_pub = ros_node_->create_publisher<hrim_generic_msgs::msg::Power>(topic_name_power);
    RCLCPP_INFO(ros_node_->get_logger(), "creating %s publisher topic", topic_name_power.c_str());

    status_pub = ros_node_->create_publisher<hrim_generic_msgs::msg::Status>(topic_name_status);
    RCLCPP_INFO(ros_node_->get_logger(), "creating %s publisher topic", topic_name_status.c_str());

    std::function<void( std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<hrim_generic_srvs::srv::SimulationURDF::Request>,
                        std::shared_ptr<hrim_generic_srvs::srv::SimulationURDF::Response>)> cb_SimulationURDF_function = std::bind(
          &RobotiqGripperPlugin::URDFService, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);
    sim_urdf_srv_ = ros_node_->create_service<hrim_generic_srvs::srv::SimulationURDF>(service_name_simurdf, cb_SimulationURDF_function);
    RCUTILS_LOG_INFO_NAMED(ros_node_->get_name(), "creating service called: %s ", service_name_simurdf.c_str());

    std::function<void( std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<hrim_generic_srvs::srv::Simulation3D::Request>,
                        std::shared_ptr<hrim_generic_srvs::srv::Simulation3D::Response>)> cb_Simulation3D_function = std::bind(
          &RobotiqGripperPlugin::Sim3DService, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);
    sim_3d_srv_ = ros_node_->create_service<hrim_generic_srvs::srv::Simulation3D>(service_name_sim3d, cb_Simulation3D_function);
    RCUTILS_LOG_INFO_NAMED(ros_node_->get_name(), "creating service called: %s ", service_name_sim3d.c_str());

    std::function<void( const std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<hrim_actuator_gripper_srvs::srv::SpecsFingerGripper::Request>,
                        std::shared_ptr<hrim_actuator_gripper_srvs::srv::SpecsFingerGripper::Response>)> cb_SpecsFingerGripper_function = std::bind(
          &RobotiqGripperPlugin::SpecsFingerGripperService, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);
    specs_srv_ = ros_node_->create_service<hrim_actuator_gripper_srvs::srv::SpecsFingerGripper>(service_name_specs, cb_SpecsFingerGripper_function);

    state_comm_pub = ros_node_->create_publisher<hrim_generic_msgs::msg::StateCommunication>(topic_name_state_comm,
                  rmw_qos_profile_default);
    RCLCPP_ERROR(ros_node_->get_logger(), "creating %s publisher topic", topic_name_state_comm.c_str());

    std::function<void( std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<hrim_generic_srvs::srv::SpecsCommunication::Request>,
                        std::shared_ptr<hrim_generic_srvs::srv::SpecsCommunication::Response>)> cb_SpecsCommunication_function = std::bind(
          &RobotiqGripperPlugin::SpecsCommunicationService, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);
    specs_comm_srv_ = ros_node_->create_service<hrim_generic_srvs::srv::SpecsCommunication>(service_name_specs_comm, cb_SpecsCommunication_function);

    timer_status_ = ros_node_->create_wall_timer(
        1s, std::bind(&RobotiqGripperPlugin::timer_status_msgs, this));
    timer_power_ = ros_node_->create_wall_timer(
        1s, std::bind(&RobotiqGripperPlugin::timer_power_msgs, this));
    timer_comm_ = ros_node_->create_wall_timer(
        1s, std::bind(&RobotiqGripperPlugin::timer_comm_msgs, this));
    timer_gripper_status_ = ros_node_->create_wall_timer(
        100ms, std::bind(&RobotiqGripperPlugin::timer_gripper_status_msgs, this));
  }

  // void RobotiqGripperPlugin::readfullFile(std::string file_to_read, hrim_generic_srvs::srv::Simulation3D& msg_sim_3d)
  // {
  //   std::string robotiq_140_description_folder = ament_index_cpp::get_package_share_directory("robotiq_140_gripper_description");
  //
  //   gzmsg << "readfullFile " << robotiq_140_description_folder + file_to_read << std::endl;
  //
  //   std::ifstream ifs(robotiq_140_description_folder + file_to_read, std::ios::binary|std::ios::ate);
  //
  //   if(!ifs.is_open()){
  //     gzmsg << "Error reading file " << robotiq_140_description_folder + file_to_read << std::endl;
  //     return;
  //   }
  //
  //   std::ifstream::pos_type pos = ifs.tellg();
  //
  //   msg_sim_3d.model.resize(pos);
  //   ifs.seekg(0, std::ios::beg);
  //   ifs.read((char *)&msg_sim_3d.model[0], pos);
  //   ifs.close();
  // }
  void RobotiqGripperPlugin::timer_power_msgs()
  {
    hrim_generic_msgs::msg::Power power_msg;
    gazebo::common::Time cur_time = this->model->GetWorld()->SimTime();
    power_msg.header.stamp.sec = cur_time.sec;
    power_msg.header.stamp.nanosec = cur_time.nsec;
    power_msg.voltage = 48.0;
    power_msg.current_consumption = 0.1;
    power_msg.power_consumption = power_msg.current_consumption*power_msg.voltage;
    power_pub->publish(power_msg);
  }

  void RobotiqGripperPlugin::timer_status_msgs()
  {
    hrim_generic_msgs::msg::Status status_msg;
    gazebo::common::Time cur_time = this->model->GetWorld()->SimTime();
    status_msg.header.stamp.sec = cur_time.sec;
    status_msg.header.stamp.nanosec = cur_time.nsec;
    status_pub->publish(status_msg);
  }

  void RobotiqGripperPlugin::timer_gripper_status_msgs()
  {
    hrim_actuator_gripper_msgs::msg::StateFingerGripper state_gripper_finger_msg;
    gazebo::common::Time cur_time = this->model->GetWorld()->SimTime();
    state_gripper_finger_msg.header.stamp.sec = cur_time.sec;
    state_gripper_finger_msg.header.stamp.nanosec = cur_time.nsec;
    state_gripper_finger_msg.angular_position = joint_v_.front()->Position(0);
    state_gripper_finger_msg.linear_position = 0;

    gripper_finger_state_pub->publish(state_gripper_finger_msg);

    hrim_actuator_gripper_msgs::msg::StateGripper state_gripper_msg;
    state_gripper_msg.header.stamp.sec = cur_time.sec;
    state_gripper_msg.header.stamp.nanosec = cur_time.nsec;
    state_gripper_msg.on_off = true;
    gripper_state_pub->publish(state_gripper_msg);
  }
  //
  // void RobotiqGripperPlugin::timer_specs_msgs()
  // {
  //   hrim_actuator_gripper_srvs::srv::SpecsFingerGripper specs_msg;
  //   gazebo::common::Time cur_time = this->model->GetWorld()->SimTime();
  //   res->header.stamp.sec = cur_time.sec;
  //   res->header.stamp.nanosec = cur_time.nsec;
  //   res->min_force = MIN_FORCE; // Minimum gripping force [N]
  //   res->max_force = MAX_FORCE; // Maximun gripping force [N]
  //
  //   res->max_payload = MAX_FORCE;   // Maximum recommended payload [kg]
  //
  //   res->min_speed = MIN_SPEED;  // Minimum closing speed [mm/s]
  //   res->max_speed = MAX_SPEED;  // Maximum  closing speed [mm/s]
  //
  //   res->max_acceleration = MAX_ACCELERATION;
  //
  //   res->max_length = MAX_LENGHT; // Maximum permitted finger length [mm]
  //   res->max_angle = MAX_ANGLE;  // Maximum permitted finger angle [rad]
  //
  //   res->repeatability = REPEATABILITY;
  //
  //   specs_pub->publish(specs_msg);
  // }

  void RobotiqGripperPlugin::timer_comm_msgs()
  {
    gazebo::common::Time cur_time = this->model->GetWorld()->SimTime();

    hrim_generic_msgs::msg::StateCommunication state_comm_msg;
    state_comm_msg.header.stamp.sec = cur_time.sec;
    state_comm_msg.header.stamp.nanosec = cur_time.nsec;
    state_comm_pub->publish(state_comm_msg);
  }


  void RobotiqGripperPlugin::SpecsCommunicationService(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<hrim_generic_srvs::srv::SpecsCommunication::Request> req,
      std::shared_ptr<hrim_generic_srvs::srv::SpecsCommunication::Response> res)
  {

  }

  void RobotiqGripperPlugin::SpecsFingerGripperService(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<hrim_actuator_gripper_srvs::srv::SpecsFingerGripper::Request> req,
      std::shared_ptr<hrim_actuator_gripper_srvs::srv::SpecsFingerGripper::Response> res)
  {
    res->min_force = MIN_FORCE; // Minimum gripping force [N]
    res->max_force = MAX_FORCE; // Maximun gripping force [N]

    res->max_payload = MAX_FORCE;   // Maximum recommended payload [kg]

    res->min_speed = MIN_SPEED;  // Minimum closing speed [mm/s]
    res->max_speed = MAX_SPEED;  // Maximum  closing speed [mm/s]

    res->max_acceleration = MAX_ACCELERATION;

    res->max_length = MAX_LENGHT; // Maximum permitted finger length [mm]
    res->max_angle = joint_v_.front()->UpperLimit();  // Maximum permitted finger angle [rad]

    res->repeatability = REPEATABILITY;
  }


  void RobotiqGripperPlugin::IDService(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<hrim_generic_srvs::srv::ID::Request> req,
      std::shared_ptr<hrim_generic_srvs::srv::ID::Response> res)
  {
    (void)request_header;
    (void)req;

    res->device_kind_id = hrim_generic_srvs::srv::ID::Response::HRIM_ACTUATOR;
    res->hros_version = "Ardent";
    res->hrim_version = "Anboto";
  }

  void RobotiqGripperPlugin::URDFService(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<hrim_generic_srvs::srv::SimulationURDF::Request> req,
      std::shared_ptr<hrim_generic_srvs::srv::SimulationURDF::Response> res)
  {
    (void)request_header;
    (void)req;

    // std::ifstream t(urdf_file);
    // std::string str;
    //
    // t.seekg(0, std::ios::end);
    // str.reserve(t.tellg());
    // t.seekg(0, std::ios::beg);
    //
    // str.assign((std::istreambuf_iterator<char>(t)),
    //             std::istreambuf_iterator<char>());
    //
    // res->urdf_model = str;
  }

  void RobotiqGripperPlugin::Sim3DService(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<hrim_generic_srvs::srv::Simulation3D::Request> req,
      std::shared_ptr<hrim_generic_srvs::srv::Simulation3D::Response> res)
  {
    (void)request_header;
    (void)req;

    // std::ifstream ifs(stl_file, std::ios::binary|std::ios::ate);
    // std::ifstream::pos_type pos = ifs.tellg();
    //
    // res->model.resize(pos);
    // ifs.seekg(0, std::ios::beg);
    // ifs.read(&res->model[0], pos);
  }

  GZ_REGISTER_MODEL_PLUGIN(RobotiqGripperPlugin)
}
