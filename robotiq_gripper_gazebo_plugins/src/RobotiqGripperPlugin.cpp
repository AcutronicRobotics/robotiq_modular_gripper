#include <string>
#include <vector>

#include "robotiq_gripper_gazebo_plugins/RobotiqGripperPlugin.h"

using gazebo::RobotiqGripperPlugin;

#define DEFAULT_FORCES_ANGLE_TOLERANCE 120
#define DEFAULT_UPDATE_RATE 5
#define DEFAULT_MAX_GRIP_COUNT 10
#define DEFAULT_RELEASE_TOLERANCE 0.005
#define DEFAULT_DISABLE_COLLISIONS_ON_ATTACH false

namespace gazebo{
  RobotiqGripperPlugin::RobotiqGripperPlugin(){
    printf("RobotiqGripperPlugin\n");
    InitValues(); //GazeboGraspFix
  }

  RobotiqGripperPlugin::~RobotiqGripperPlugin(){
    this->update_connection.reset();
    if (this->transp_node) this->transp_node->Fini();
    this->transp_node.reset();
  }

  void RobotiqGripperPlugin::Init()
  {
    this->prevUpdateTime = common::Time::GetWallTime();
  }

  void RobotiqGripperPlugin::InitValues()
  {
    gazebo::common::Console::SetQuiet(false);

    // float timeDiff=0.25;
    // this->releaseTolerance=0.005;
    // this->updateRate = common::Time(0, common::Time::SecToNano(timeDiff));
    this->prevUpdateTime = common::Time::GetWallTime();
    //float graspedSecs=2;
    //this->maxGripCount=floor(graspedSecs/timeDiff);
    //this->gripCountThreshold=floor(this->maxGripCount/2);
    this->transp_node = transport::NodePtr(new transport::Node());
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
      if(this->model->GetName() == "hande"){
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
      }else if(this->model->GetName() == "robotiq_85"){
        if (request->goal_velocity >= MinVelocity_s85 && request->goal_velocity <= MaxVelocity_s85){
          target_velocity = request->goal_velocity / radius_s85;
        }else if (request->goal_velocity < MinVelocity_s85){
          target_velocity = MinVelocity_s85 / radius_s85;
          RCLCPP_INFO(node->get_logger(), "Minimum value exceeded, target velocity changed to its minimum value: %lf mm/s", MinVelocity_s85);
        }else if (request->goal_velocity > MaxVelocity_s85){
          target_velocity = MaxVelocity_s85 / radius_s85;
          RCLCPP_INFO(node->get_logger(), "maximum value exceeded, target velocity changed to its maximum value: %lf mm/s", MaxVelocity_s85);
        }
      }else if(this->model->GetName() == "robotiq_140"){
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
    gzmsg << "Loading Robotiq gripper plugin with grasp-fix plugin" << std::endl;

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
      targetJoint = 0;

    interpolated_targetJoint.clear();
    executing_joints = false;
    index_executing_joints = 0;

    //this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin( std::bind(&RobotiqGripperPlugin::OnUpdate, this));

    UpdateJointPIDs();

    // ******* Grasp plugin, code adapted from https://github.com/JenniferBuehler/gazebo-pkgs
    // grasp-fix plugin +++ Read parameters and initialize fields +++
    gzmsg << "Loading grasp-fix plugin code section" << std::endl;

    physics::ModelPtr model = parent;
    this->world = model->GetWorld();

    sdf::ElementPtr disableCollisionsOnAttachElem =
      sdf->GetElement("disable_collisions_on_attach");
    if (!disableCollisionsOnAttachElem)
    {
      gzmsg << "GazeboGraspFix: Using default " <<
            DEFAULT_DISABLE_COLLISIONS_ON_ATTACH <<
            " because no <disable_collisions_on_attach> element specified." <<
            std::endl;
      this->disableCollisionsOnAttach = DEFAULT_DISABLE_COLLISIONS_ON_ATTACH;
    }
    else
    {
      std::string str = disableCollisionsOnAttachElem->Get<std::string>();
      bool bVal = false;
      if ((str == "true") || (str == "1"))  bVal = true;
      this->disableCollisionsOnAttach = bVal;
      gzmsg << "GazeboGraspFix: Using disable_collisions_on_attach " <<
            this->disableCollisionsOnAttach << std::endl;
    }

    sdf::ElementPtr forcesAngleToleranceElem =
      sdf->GetElement("forces_angle_tolerance");
    if (!forcesAngleToleranceElem)
    {
      gzmsg << "GazeboGraspFix: Using default tolerance of " <<
            DEFAULT_FORCES_ANGLE_TOLERANCE <<
            " because no <forces_angle_tolerance> element specified." <<
            std::endl;
      this->forcesAngleTolerance = DEFAULT_FORCES_ANGLE_TOLERANCE * M_PI / 180;
    }
    else
    {
      this->forcesAngleTolerance =
        forcesAngleToleranceElem->Get<float>() * M_PI / 180;
    }

    sdf::ElementPtr updateRateElem = sdf->GetElement("update_rate");
    double _updateSecs;
    if (!updateRateElem)
    {
      gzmsg << "GazeboGraspFix: Using  " << DEFAULT_UPDATE_RATE <<
            " because no <updateRate_tag> element specified." << std::endl;
      _updateSecs = 1.0 / DEFAULT_UPDATE_RATE;
    }
    else
    {
      int _rate = updateRateElem->Get<int>();
      double _updateRate = _rate;
      _updateSecs = 1.0 / _updateRate;
      gzmsg << "GazeboGraspFix: Using update rate " << _rate << std::endl;
    }
    this->updateRate = common::Time(0, common::Time::SecToNano(_updateSecs));

    sdf::ElementPtr maxGripCountElem = sdf->GetElement("max_grip_count");
    if (!maxGripCountElem)
    {
      gzmsg << "GazeboGraspFix: Using  " << DEFAULT_MAX_GRIP_COUNT <<
            " because no <max_grip_count> element specified." << std::endl;
      this->maxGripCount = DEFAULT_MAX_GRIP_COUNT;
    }
    else
    {
      this->maxGripCount = maxGripCountElem->Get<int>();
      gzmsg << "GazeboGraspFix: Using max_grip_count "
            << this->maxGripCount << std::endl;
    }

    sdf::ElementPtr gripCountThresholdElem =
      sdf->GetElement("grip_count_threshold");
    if (!gripCountThresholdElem)
    {
      this->gripCountThreshold = floor(this->maxGripCount / 2.0);
      gzmsg << "GazeboGraspFix: Using  " << this->gripCountThreshold <<
            " because no <grip_count_threshold> element specified." <<
            std::endl;
    }
    else
    {
      this->gripCountThreshold = gripCountThresholdElem->Get<int>();
      gzmsg << "GazeboGraspFix: Using grip_count_threshold " <<
            this->gripCountThreshold << std::endl;
    }

    sdf::ElementPtr releaseToleranceElem = sdf->GetElement("release_tolerance");
    if (!releaseToleranceElem)
    {
      gzmsg << "GazeboGraspFix: Using  " << DEFAULT_RELEASE_TOLERANCE <<
            " because no <release_tolerance> element specified." << std::endl;
      this->releaseTolerance = DEFAULT_RELEASE_TOLERANCE;
    }
    else
    {
      this->releaseTolerance = releaseToleranceElem->Get<float>();
      gzmsg << "GazeboGraspFix: Using release_tolerance " <<
            this->releaseTolerance << std::endl;
    }

    // will contain all names of collision entities involved from all arms
    std::vector<std::string> collisionNames;

    sdf::ElementPtr armElem = sdf->GetElement("arm");
    if (!armElem)
    {
      gzerr << "GazeboGraspFix: Cannot load the GazeboGraspFix without any <arm> declarations"
            << std::endl;
      return;
    }
    // add all arms:
    for (; armElem != NULL; armElem = armElem->GetNextElement("arm"))
    {
      sdf::ElementPtr armNameElem = armElem->GetElement("arm_name");
      sdf::ElementPtr handLinkElem = armElem->GetElement("palm_link");
      sdf::ElementPtr fingerLinkElem = armElem->GetElement("gripper_link");

      if (!handLinkElem || !fingerLinkElem || !armNameElem)
      {
        gzerr << "ERROR: GazeboGraspFix: Cannot use a GazeboGraspFix arm because "
              << "not all of <arm_name>, <palm_link> and <gripper_link> elements specified in URDF/SDF. Skipping."
              << std::endl;
        continue;
      }

      std::string armName = armNameElem->Get<std::string>();
      std::string palmName = handLinkElem->Get<std::string>();

      // collect all finger names:
      std::vector<std::string> fingerLinkNames;
      for (; fingerLinkElem != NULL;
           fingerLinkElem = fingerLinkElem->GetNextElement("gripper_link"))
      {
        std::string linkName = fingerLinkElem->Get<std::string>();
        fingerLinkNames.push_back(linkName);
      }

      // add new gripper
      if (grippers.find(armName) != grippers.end())
      {
        gzerr << "GazeboGraspFix: Arm named " << armName <<
              " was already added, cannot add it twice." << std::endl;
      }
      GazeboGraspGripper &gripper = grippers[armName];
      std::map<std::string, physics::CollisionPtr> _collisions;
      if (!gripper.Init(parent, armName, palmName, fingerLinkNames,
                        disableCollisionsOnAttach, _collisions))
      {
        gzerr << "GazeboGraspFix: Could not initialize arm " << armName << ". Skipping."
              << std::endl;
        grippers.erase(armName);
        continue;
      }
      // add all the grippers's collision elements
      for (std::map<std::string, physics::CollisionPtr>::iterator collIt =
             _collisions.begin();
           collIt != _collisions.end(); ++collIt)
      {
        const std::string &collName = collIt->first;
        //physics::CollisionPtr& coll=collIt->second;
        std::map<std::string, std::string>::iterator collIter = this->collisions.find(
              collName);
        if (collIter !=
            this->collisions.end())   //this collision was already added before
        {
          gzwarn << "GazeboGraspFix: Adding Gazebo collision link element " << collName <<
                 " multiple times, the grasp plugin may not work properly" << std::endl;
          continue;
        }
        gzmsg << "GazeboGraspFix: Adding collision scoped name " << collName <<
              std::endl;
        this->collisions[collName] = armName;
        collisionNames.push_back(collName);
      }
    }

    if (grippers.empty())
    {
      gzerr << "ERROR: GazeboGraspFix: Cannot use a GazeboGraspFix because "
            << "no arms were configured successfully. Plugin will not work." << std::endl;
      return;
    }

    // grasp-fix plugin +++ start up things +++

    physics::PhysicsEnginePtr physics = this->world->Physics();
    this->transp_node->Init(this->world->Name());
    physics::ContactManager *contactManager = physics->GetContactManager();
    contactManager->PublishContacts();  // TODO not sure this is required?

    std::string topic = contactManager->CreateFilter(model->GetScopedName(),
                        collisionNames);
    if (!this->contactSub)
    {
      gzmsg << "Subscribing contact manager to topic " << topic << std::endl;
      bool latching = false;
      this->contactSub = this->transp_node->Subscribe(topic, &RobotiqGripperPlugin::OnContact,
                         this, latching);
    }

    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(
                          &RobotiqGripperPlugin::OnUpdate, this));
    //this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin( std::bind(&RobotiqGripperPlugin::OnUpdate, this));

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

  void RobotiqGripperPlugin::OnUpdate(){ //UpdatePIDControl + interpolation + gripper_fix gripping plugin
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

////////////////////////////////////////////////////////////////////////////////
/**
 * Helper class to encapsulate a collision information.
 * One contact has two bodies, and only
 * the ones where one of the bodies is a gripper link are considered.
 * Each contact consists of a *list* of forces with their own origin/position each
 * (e.g. when the object and gripper are colliding at several places).
 * The averages of each contact's force vectors along with their origins are
 * *accumulated* in the given Vector3 \e pos and \eforce objects.
 * The number of additions is stored in \e sum.
 * This is to get the average force application over time between link and object.
 *
 * \author Jennifer Buehler
 */
class RobotiqGripperPlugin::CollidingPoint
{
  public:
    CollidingPoint(): sum(0) {}
    CollidingPoint(const CollidingPoint &o):
      gripperName(o.gripperName),
      collLink(o.collLink),
      collObj(o.collObj),
      force(o.force),
      pos(o.pos),
      objPos(o.objPos),
      sum(o.sum) {}

    // Name of the gripper/arm involved in contact point
    // This is not the specific link, but the name of the
    // whole gripper
    std::string gripperName;

    // the collision
    physics::CollisionPtr collLink, collObj;

    // average force vector of the colliding point
    gz_math_Vector3d force;

    // position (relative to reference frame of gripper
    // collision surface) where the contact happens on collision surface
    gz_math_Vector3d pos;

    // position (relative to reference frame of *gripper* collision surface)
    // where the object center is located during collision.
    gz_math_Vector3d objPos;

    // sum of force and pose (they are actually summed
    // up from several contact points).
    // Divide both \e force and \e pos by this to obtain average
    int sum;
};

  GZ_REGISTER_MODEL_PLUGIN(RobotiqGripperPlugin)
}

void RobotiqGripperPlugin::OnContact(const ConstContactsPtr &_msg)
{
    //std::cout<<"CONTACT! "<<std::endl;//<<_contact<<std::endl;
    // for all contacts...
    for (int i = 0; i < _msg->contact_size(); ++i) {

        physics::CollisionPtr collision1 = boost::dynamic_pointer_cast<physics::Collision>(
                this->world->EntityByName(_msg->contact(i).collision1()));
        physics::CollisionPtr collision2 = boost::dynamic_pointer_cast<physics::Collision>(
                this->world->EntityByName(_msg->contact(i).collision2()));

        if ((collision1 && !collision1->IsStatic()) && (collision2 && !collision2->IsStatic()))
        {
            std::string name1 = collision1->GetScopedName();
            std::string name2 = collision2->GetScopedName();

            //std::cout<<"OBJ CONTACT! "<<name1<<" / "<<name2<<std::endl;
            int count = _msg->contact(i).position_size();

            // Check to see if the contact arrays all have the same size.
            if ((count != _msg->contact(i).normal_size()) ||
                (count != _msg->contact(i).wrench_size()) ||
                (count != _msg->contact(i).depth_size()))
            {
                gzerr << "GazeboGraspFix: Contact message has invalid array sizes\n"<<std::endl;
                continue;
            }

            std::string collidingObjName, collidingLink, gripperOfCollidingLink;
            physics::CollisionPtr linkCollision;
            physics::CollisionPtr objCollision;

            physics::Contact contact;
            contact = _msg->contact(i);

            if (contact.count<1)
            {
                std::cerr<<"ERROR: GazeboGraspFix: Not enough forces given for contact of ."<<name1<<" / "<<name2<<std::endl;
                continue;
            }

            // all force vectors which are part of this contact
            std::vector<gz_math_Vector3d> force;
            
            // find out which part of the colliding entities is the object, *not* the gripper,
            // and copy all the forces applied to it into the vector 'force'.
            std::map<std::string,std::string>::const_iterator gripperCollIt = this->collisions.find(name2);
            if (gripperCollIt != this->collisions.end())
            {   // collision 1 is the object
                collidingObjName=name1;
                collidingLink=name2;
                linkCollision=collision2;
                objCollision=collision1;
                gripperOfCollidingLink=gripperCollIt->second;
                for (int k=0; k<contact.count; ++k)
                    force.push_back(contact.wrench[k].body1Force);
            }
            else if ((gripperCollIt=this->collisions.find(name1)) != this->collisions.end())
            {   // collision 2 is the object
                collidingObjName=name2;
                collidingLink=name1;
                linkCollision=collision1;
                objCollision=collision2;
                gripperOfCollidingLink=gripperCollIt->second;
                for (int k=0; k<contact.count; ++k)
                    force.push_back(contact.wrench[k].body2Force);
            }

            gz_math_Vector3d avgForce;
            // compute average/sum of the forces applied on the object
            for (unsigned int k=0; k<force.size(); ++k){
                avgForce+=force[k];
            }    
            avgForce/=force.size();

            gz_math_Vector3d avgPos;
            // compute center point (average pose) of all the origin positions of the forces appied
            for (int k=0; k<contact.count; ++k) avgPos+=contact.positions[k];
            avgPos/=contact.count;

            // now, get average pose relative to the colliding link
            gz_math_Pose3d linkWorldPose=linkCollision->GetLink()->WorldPose();

            // To find out the collision point relative to the Link's local coordinate system, first get the Poses as 4x4 matrices


            gz_math_Matrix4d worldToLink(linkWorldPose.Rot());
            worldToLink.SetTranslation(linkWorldPose.Pos());
            
            gz_math_Matrix4d worldToContact=gz_math_Matrix4d::Identity;
            //we can assume that the contact has identity rotation because we don't care about its orientation.
            //We could always set another rotation here too.
            worldToContact.SetTranslation(avgPos);

            // now, worldToLink * contactInLocal = worldToContact
            // hence, contactInLocal = worldToLink.Inv * worldToContact
            gz_math_Matrix4d worldToLinkInv = worldToLink.Inverse();
            gz_math_Matrix4d contactInLocal = worldToLinkInv * worldToContact;
            gz_math_Vector3d contactPosInLocal = contactInLocal.Translation();
            
            //std::cout<<"---------"<<std::endl;    
            //std::cout<<"CNT in loc: "<<contactPosInLocal.x<<","<<contactPosInLocal.y<<","<<contactPosInLocal.z<<std::endl;

            /*gz_math_Vector3d sDiff=avgPos-linkWorldPose.pos;
            std::cout<<"SIMPLE trans: "<<sDiff.x<<","<<sDiff.y<<","<<sDiff.z<<std::endl;
            std::cout<<"coll world pose: "<<linkWorldPose.pos.x<<", "<<linkWorldPose.pos.y<<", "<<linkWorldPose.pos.z<<std::endl; 
            std::cout<<"contact avg pose: "<<avgPos.x<<", "<<avgPos.y<<", "<<avgPos.z<<std::endl; 
            gz_math_Vector3d lX=linkWorldPose.rot.GetXAxis();
            gz_math_Vector3d lY=linkWorldPose.rot.GetYAxis();
            gz_math_Vector3d lZ=linkWorldPose.rot.GetZAxis();
    
            std::cout<<"World ori: "<<linkWorldPose.rot.x<<","<<linkWorldPose.rot.y<<","<<linkWorldPose.rot.z<<","<<linkWorldPose.rot.w<<std::endl;
            std::cout<<"x axis: "<<lX.x<<","<<lX.y<<","<<lX.z<<std::endl;
            std::cout<<"y axis: "<<lY.x<<","<<lY.y<<","<<lY.z<<std::endl;
            std::cout<<"z axis: "<<lZ.x<<","<<lZ.y<<","<<lZ.z<<std::endl;*/

            // now, get the pose of the object and compute it's relative position to the collision surface.
            gz_math_Pose3d objWorldPose = objCollision->GetLink()->WorldPose();



            gz_math_Matrix4d worldToObj(objWorldPose.Rot());
            worldToObj.SetTranslation(objWorldPose.Pos());
    
            gz_math_Matrix4d objInLocal = worldToLinkInv * worldToObj;
            gz_math_Vector3d objPosInLocal = objInLocal.Translation();

            {
                boost::mutex::scoped_lock lock(this->mutexContacts);
                CollidingPoint& p = this->contacts[collidingObjName][collidingLink];  // inserts new entry if doesn't exist
                p.gripperName=gripperOfCollidingLink;
                p.collLink=linkCollision;
                p.collObj=objCollision;
                p.force+=avgForce;
                p.pos+=contactPosInLocal;
                p.objPos+=objPosInLocal;
                p.sum++;
            }
            //std::cout<<"Average force of contact= "<<avgForce.x<<", "<<avgForce.y<<", "<<avgForce.z<<" out of "<<force.size()<<" vectors."<<std::endl;
        }
    }
}
