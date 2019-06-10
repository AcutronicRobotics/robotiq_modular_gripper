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

      if((int)this->joint_type == 1088){ // prismatic
        target_pose = request->goal_linearposition;
      }
      else if((int)this->joint_type == 576){ // revolute
        target_pose = request->goal_angularposition;
      }
      else{
        RCLCPP_ERROR(node->get_logger(), "joint_finger type not supported");
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
      if((int)this->joint_type == 1088){ // prismatic (e.g. S50 gripper)
        if (request->goal_velocity >= MinVelocity && request->goal_velocity <= MaxVelocity){
          target_velocity = request->goal_velocity;
        }else if (request->goal_velocity < MinVelocity){
          target_velocity = MinVelocity;
          RCLCPP_INFO(node->get_logger(), "Minimum value exceeded, target velocity changed to its minimum value: %lf mm/s", MinVelocity);
        }else if (request->goal_velocity > MaxVelocity){
          target_velocity = MaxVelocity;
          RCLCPP_INFO(node->get_logger(), "maximum value exceeded, target velocity changed to its maximum value: %lf mm/s", MaxVelocity);
        }
        target_velocity *= 0.001; // to m/s
      }
      else if((int)this->joint_type == 576){ // revolute (e.g. S140 and S85 grippers)
        if (request->goal_velocity >= MinVelocity && request->goal_velocity <= MaxVelocity){
          target_velocity = request->goal_velocity / radius;
        }else if (request->goal_velocity < MinVelocity){
          target_velocity = MinVelocity / radius;
          RCLCPP_INFO(node->get_logger(), "Minimum value exceeded, target velocity changed to its minimum value: %lf mm/s", MinVelocity);
        }else if (request->goal_velocity > MaxVelocity){
          target_velocity = MaxVelocity / radius;
          RCLCPP_INFO(node->get_logger(), "maximum value exceeded, target velocity changed to its maximum value: %lf mm/s", MaxVelocity);
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

    id_srv_ = node->create_service<hrim_generic_srvs::srv::ID>(service_name_id, cb_id_function);
    RCUTILS_LOG_INFO_NAMED(node->get_name(), "creating service called: %s ", service_name_id.c_str());

    power_pub = node->create_publisher<hrim_generic_msgs::msg::Power>(topic_name_power);
    RCLCPP_INFO(node->get_logger(), "creating %s publisher topic", topic_name_power.c_str());

    status_pub = node->create_publisher<hrim_generic_msgs::msg::Status>(topic_name_status);
    RCLCPP_INFO(node->get_logger(), "creating %s publisher topic", topic_name_status.c_str());

    std::function<void( std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<hrim_generic_srvs::srv::SimulationURDF::Request>,
                        std::shared_ptr<hrim_generic_srvs::srv::SimulationURDF::Response>)> cb_SimulationURDF_function = std::bind(
          &RobotiqGripperPlugin::URDFService, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);
    sim_urdf_srv_ = node->create_service<hrim_generic_srvs::srv::SimulationURDF>(service_name_simurdf, cb_SimulationURDF_function);
    RCUTILS_LOG_INFO_NAMED(node->get_name(), "creating service called: %s ", service_name_simurdf.c_str());

    std::function<void( std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<hrim_generic_srvs::srv::Simulation3D::Request>,
                        std::shared_ptr<hrim_generic_srvs::srv::Simulation3D::Response>)> cb_Simulation3D_function = std::bind(
          &RobotiqGripperPlugin::Sim3DService, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);
    sim_3d_srv_ = node->create_service<hrim_generic_srvs::srv::Simulation3D>(service_name_sim3d, cb_Simulation3D_function);
    RCUTILS_LOG_INFO_NAMED(node->get_name(), "creating service called: %s ", service_name_sim3d.c_str());

    std::function<void( const std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<hrim_actuator_gripper_srvs::srv::SpecsFingerGripper::Request>,
                        std::shared_ptr<hrim_actuator_gripper_srvs::srv::SpecsFingerGripper::Response>)> cb_SpecsFingerGripper_function = std::bind(
          &RobotiqGripperPlugin::SpecsFingerGripperService, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);
    specs_srv_ = node->create_service<hrim_actuator_gripper_srvs::srv::SpecsFingerGripper>(service_name_specs, cb_SpecsFingerGripper_function);

    state_comm_pub = node->create_publisher<hrim_generic_msgs::msg::StateCommunication>(topic_name_state_comm,
                  rmw_qos_profile_default);
    RCLCPP_ERROR(node->get_logger(), "creating %s publisher topic", topic_name_state_comm.c_str());

    std::function<void( std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<hrim_generic_srvs::srv::SpecsCommunication::Request>,
                        std::shared_ptr<hrim_generic_srvs::srv::SpecsCommunication::Response>)> cb_SpecsCommunication_function = std::bind(
          &RobotiqGripperPlugin::SpecsCommunicationService, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);
    specs_comm_srv_ = node->create_service<hrim_generic_srvs::srv::SpecsCommunication>(service_name_specs_comm, cb_SpecsCommunication_function);

    timer_status_ = node->create_wall_timer(
        1s, std::bind(&RobotiqGripperPlugin::timer_status_msgs, this));
    timer_power_ = node->create_wall_timer(
        1s, std::bind(&RobotiqGripperPlugin::timer_power_msgs, this));
    timer_comm_ = node->create_wall_timer(
        1s, std::bind(&RobotiqGripperPlugin::timer_comm_msgs, this));
    timer_gripper_status_ = node->create_wall_timer(
        100ms, std::bind(&RobotiqGripperPlugin::timer_gripper_status_msgs, this));
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

    createGenericTopics(node_name);
    createTopicAndService(node_name);

    if(this->sdf->HasElement("kp"))
      this->kp = this->sdf->GetElement("kp")->Get<double>();
    if(this->sdf->HasElement("ki"))
      this->ki = this->sdf->GetElement("ki")->Get<double>();
    if(this->sdf->HasElement("kd"))
      this->kd = this->sdf->GetElement("kd")->Get<double>();

    // velocity related tags
    if(this->sdf->HasElement("min_velocity")){
      this->MinVelocity = this->sdf->GetElement("min_velocity")->Get<double>();
    }else{
      RCLCPP_ERROR(node->get_logger(), "No min_velocity element.");
      node.reset();
      return;
    }
    if(this->sdf->HasElement("max_velocity")){
      this->MaxVelocity = this->sdf->GetElement("max_velocity")->Get<double>();
    }else{
      RCLCPP_ERROR(node->get_logger(), "No max_velocity element.");
      node.reset();
      return;
    }
    if(this->sdf->HasElement("radius")){
      this->radius = this->sdf->GetElement("radius")->Get<double>();
    }else{
      RCLCPP_INFO(node->get_logger(), "No radius element found. Angular movement radius must be set for grippers with revolute joints.");
    }

    sdf::ElementPtr joint_elem = this->sdf->GetElement("joint");
    while(joint_elem){
      auto joint_name = joint_elem->Get<std::string>();
      auto joint = model->GetJoint(joint_name);

      if(joint_name.find("joint_finger")){
        this->joint_type = this->model->GetJoint(joint_name)->GetType();
      }

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

    if((int)this->joint_type == -1){
      RCLCPP_ERROR(node->get_logger(), "joint_finger is missing, the gripper will not work");
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

  void RobotiqGripperPlugin::createTopicAndService(std::string node_name)
  {
    std::string topic_name_gripper_state = std::string(node_name) + "/gripperstate";
    gripper_state_pub = node->create_publisher<hrim_actuator_gripper_msgs::msg::StateGripper>(topic_name_gripper_state,
                  rmw_qos_profile_default);

    std::string fingerstate = std::string(node_name) + "/fingerstate";
    RCLCPP_INFO(node->get_logger(), "creating %s publisher ", fingerstate.c_str());
    fingerstatePublisher = node->create_publisher<hrim_actuator_gripper_msgs::msg::StateFingerGripper>(fingerstate, rmw_qos_profile_default);

    timer_fingerstate = node->create_wall_timer(100ms, std::bind(&RobotiqGripperPlugin::timer_fingerstate_msgs, this));

    std::string fingercontrol = std::string(node_name) + "/goal";
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

    if((int)this->joint_type == 1088){
      // prismatic
      fingerstateMsg.linear_position = jointsVec.front()->Position(0);
      fingerstateMsg.angular_position = 0;
    }
    else if((int)this->joint_type == 576){
      // revolute
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
    gazebo::common::Time cur_time = this->model->GetWorld()->SimTime();

    hrim_actuator_gripper_msgs::msg::StateGripper state_gripper_msg;
    state_gripper_msg.header.stamp.sec = cur_time.sec;
    state_gripper_msg.header.stamp.nanosec = cur_time.nsec;
    state_gripper_msg.on_off = true;
    gripper_state_pub->publish(state_gripper_msg);
  }

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
    res->max_angle = jointsVec.front()->UpperLimit();  // Maximum permitted finger angle [rad]

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
    res->hros_version = "Dashing";
    res->hrim_version = "Coliza";
    res->device_name = "Gripper";
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
