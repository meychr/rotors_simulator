/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.M
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_gazebo_plugins/gazebo_motor_model.h"

#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"

namespace gazebo {

GazeboMotorModel::~GazeboMotorModel() {
}

void GazeboMotorModel::InitializeParams() {}

void GazeboMotorModel::Publish() {
  if (publish_speed_) {
    turning_velocity_msg_.set_data(joint_->GetVelocity(0));
    motor_velocity_pub_->Publish(turning_velocity_msg_);
  }
  if (publish_position_) {
    position_msg_.set_data(joint_->Position(0));
    motor_position_pub_->Publish(position_msg_);
  }
  if (publish_force_) {
    force_msg_.set_data(joint_->GetForce(0));
    motor_force_pub_->Publish(force_msg_);
  }
}

void GazeboMotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  model_ = _model;

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor "
             "is attached.\n";

  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow(
        "[gazebo_motor_model] Couldn't find specified joint \"" << joint_name_
                                                                << "\".");

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow(
        "[gazebo_motor_model] Couldn't find specified link \"" << link_name_
                                                               << "\".");
  base_link_name_ = "base_link";
  base_link_ = model_->GetLink(base_link_name_);
  if (base_link_ == NULL)
    gzthrow(
      "[gazebo_motor_model] Couldn't find specified base_link \"" << base_link_name_
                                                             << "\".");

  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

  if (_sdf->HasElement("turningDirection")) {
    std::string turning_direction =
        _sdf->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
      turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
      turning_direction_ = turning_direction::CCW;
    else
      gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as "
               "turningDirection.\n";
  } else
    gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or "
             "'ccw').\n";

  if (_sdf->HasElement("motorType")) {
    std::string motor_type = _sdf->GetElement("motorType")->Get<std::string>();
    if (motor_type == "velocity")
      motor_type_ = MotorType::kVelocity;
    else if (motor_type == "position")
      motor_type_ = MotorType::kPosition;
    else if (motor_type == "force") {
      motor_type_ = MotorType::kForce;
    } else
      gzerr << "[gazebo_motor_model] Please only use 'velocity', 'position' or "
               "'force' as motorType.\n";
  } else {
    gzwarn << "[gazebo_motor_model] motorType not specified, using velocity.\n";
    motor_type_ = MotorType::kVelocity;
  }

  // Set up joint control PID to control joint.
  if (motor_type_ == MotorType::kPosition) {
    if (_sdf->HasElement("joint_control_pid")) {
      sdf::ElementPtr pid = _sdf->GetElement("joint_control_pid");
      double p = 0;
      if (pid->HasElement("p"))
        p = pid->Get<double>("p");
      double i = 0;
      if (pid->HasElement("i"))
        i = pid->Get<double>("i");
      double d = 0;
      if (pid->HasElement("d"))
        d = pid->Get<double>("d");
      double iMax = 0;
      if (pid->HasElement("iMax"))
        iMax = pid->Get<double>("iMax");
      double iMin = 0;
      if (pid->HasElement("iMin"))
        iMin = pid->Get<double>("iMin");
      double cmdMax = 0;
      if (pid->HasElement("cmdMax"))
        cmdMax = pid->Get<double>("cmdMax");
      double cmdMin = 0;
      if (pid->HasElement("cmdMin"))
        cmdMin = pid->Get<double>("cmdMin");
      pids_.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
    } else {
      pids_.Init(0, 0, 0, 0, 0, 0, 0);
      gzerr << "[gazebo_motor_model] PID values not found, Setting all values "
               "to zero!\n";
    }
  } else {
    pids_.Init(0, 0, 0, 0, 0, 0, 0);
  }

  getSdfParam<std::string>(
      _sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<std::string>(
      _sdf, "windSpeedSubTopic", wind_speed_sub_topic_, wind_speed_sub_topic_);
  getSdfParam<std::string>(
      _sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
      motor_speed_pub_topic_);

  // Only publish position and force messages if a topic is specified in sdf.
  if (_sdf->HasElement("motorPositionPubTopic")) {
    publish_position_ = true;
    motor_position_pub_topic_ =
        _sdf->GetElement("motorPositionPubTopic")->Get<std::string>();
  }
  if (_sdf->HasElement("motorForcePubTopic")) {
    publish_force_ = true;
    motor_force_pub_topic_ =
        _sdf->GetElement("motorForcePubTopic")->Get<std::string>();
  }

  getSdfParam<double>(
      _sdf, "rotorDragCoefficient", rotor_drag_coefficient_,
      rotor_drag_coefficient_);
  getSdfParam<double>(
    _sdf, "rotorDragAngleCoefficient", rotor_drag_angle_coefficient_,
    rotor_drag_angle_coefficient_);
  getSdfParam<double>(
      _sdf, "rollingMomentCoefficient", rolling_moment_coefficient_,
      rolling_moment_coefficient_);
  getSdfParam<double>(
      _sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
  getSdfParam<double>(_sdf, "motorConstant", motor_constant_, motor_constant_);
  getSdfParam<double>(
      _sdf, "momentConstant", moment_constant_, moment_constant_);

  getSdfParam<double>(
      _sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
  getSdfParam<double>(
      _sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
  getSdfParam<double>(
      _sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMotorModel::OnUpdate, this, _1));

  // Create the first order filter.
  rotor_velocity_filter_.reset(
      new FirstOrderFilter<double>(
          time_constant_up_, time_constant_down_, ref_motor_input_));
}

// This gets called by the world update start event.
void GazeboMotorModel::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  Publish();
}

void GazeboMotorModel::CreatePubsAndSubs() {
  gzdbg << __PRETTY_FUNCTION__ << " called." << std::endl;

  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);
  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  // Create temporary "ConnectRosToGazeboTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);
  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;

  // ============================================ //
  //  ACTUAL MOTOR SPEED MSG SETUP (GAZEBO->ROS)  //
  // ============================================ //
  if (publish_speed_) {
    motor_velocity_pub_ = node_handle_->Advertise<gz_std_msgs::Float32>(
        "~/" + namespace_ + "/" + motor_speed_pub_topic_, 1);

    connect_gazebo_to_ros_topic_msg.set_gazebo_topic(
        "~/" + namespace_ + "/" + motor_speed_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_ros_topic(
        namespace_ + "/" + motor_speed_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_msgtype(
        gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32);
    gz_connect_gazebo_to_ros_topic_pub->Publish(
        connect_gazebo_to_ros_topic_msg, true);
  }

  // =============================================== //
  //  ACTUAL MOTOR POSITION MSG SETUP (GAZEBO->ROS)  //
  // =============================================== //

  if (publish_position_) {
    motor_position_pub_ = node_handle_->Advertise<gz_std_msgs::Float32>(
        "~/" + namespace_ + "/" + motor_position_pub_topic_, 1);

    connect_gazebo_to_ros_topic_msg.set_gazebo_topic(
        "~/" + namespace_ + "/" + motor_position_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_ros_topic(
        namespace_ + "/" + motor_position_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_msgtype(
        gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32);
    gz_connect_gazebo_to_ros_topic_pub->Publish(
        connect_gazebo_to_ros_topic_msg, true);
  }

  // ============================================ //
  //  ACTUAL MOTOR FORCE MSG SETUP (GAZEBO->ROS)  //
  // ============================================ //

  if (publish_force_) {
    motor_force_pub_ = node_handle_->Advertise<gz_std_msgs::Float32>(
        "~/" + namespace_ + "/" + motor_force_pub_topic_, 1);

    connect_gazebo_to_ros_topic_msg.set_gazebo_topic(
        "~/" + namespace_ + "/" + motor_force_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_ros_topic(
        namespace_ + "/" + motor_force_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_msgtype(
        gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32);
    gz_connect_gazebo_to_ros_topic_pub->Publish(
        connect_gazebo_to_ros_topic_msg, true);
  }

  // ============================================ //
  // = CONTROL COMMAND MSG SETUP (ROS->GAZEBO) = //
  // ============================================ //

  command_sub_ = node_handle_->Subscribe(
      "~/" + namespace_ + "/" + command_sub_topic_,
      &GazeboMotorModel::ControlCommandCallback, this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(
      namespace_ + "/" + command_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic(
      "~/" + namespace_ + "/" + command_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::COMMAND_MOTOR_SPEED);
  gz_connect_ros_to_gazebo_topic_pub->Publish(
      connect_ros_to_gazebo_topic_msg, true);

  // ============================================ //
  // ==== WIND SPEED MSG SETUP (ROS->GAZEBO) ==== //
  // ============================================ //

  /// TODO(gbmhunter): Do we need this? There is a separate Gazebo wind plugin.
  wind_speed_sub_ = node_handle_->Subscribe(
      "~/" + namespace_ + "/" + wind_speed_sub_topic_,
      &GazeboMotorModel::WindSpeedCallback, this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(
      namespace_ + "/" + wind_speed_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic(
      "~/" + namespace_ + "/" + wind_speed_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::WIND_SPEED);
  gz_connect_ros_to_gazebo_topic_pub->Publish(
      connect_ros_to_gazebo_topic_msg, true);
}

void GazeboMotorModel::ControlCommandCallback(
    GzCommandMotorInputMsgPtr& command_motor_input_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (motor_number_ > command_motor_input_msg->motor_speed_size() - 1) {
    gzerr << "You tried to access index " << motor_number_
          << " of the MotorSpeed message array which is of size "
          << command_motor_input_msg->motor_speed_size();
  }

  if (motor_type_ == MotorType::kVelocity) {
    ref_motor_input_ = std::min(
        static_cast<double>(
            command_motor_input_msg->motor_speed(motor_number_)),
        static_cast<double>(max_rot_velocity_));
  } else if (motor_type_ == MotorType::kPosition) {
    ref_motor_input_ = command_motor_input_msg->motor_speed(motor_number_);
  } else {  // if (motor_type_ == MotorType::kForce) {
    ref_motor_input_ = std::min(
        static_cast<double>(
            command_motor_input_msg->motor_speed(motor_number_)),
        static_cast<double>(max_force_));
  }
}

void GazeboMotorModel::WindSpeedCallback(GzWindSpeedMsgPtr& wind_speed_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // TODO(burrimi): Transform velocity to world frame if frame_id is set to
  // something else.
  wind_speed_W_.X() = wind_speed_msg->velocity().x();
  wind_speed_W_.Y() = wind_speed_msg->velocity().y();
  wind_speed_W_.Z() = wind_speed_msg->velocity().z();
}

double GazeboMotorModel::NormalizeAngle(double input){
      // Constrain magnitude to be max 2*M_PI.
      double wrapped = std::fmod(std::abs(input), 2*M_PI);
      wrapped = std::copysign(wrapped, input);

     // Constrain result to be element of [0, 2*pi).
     // Set angle to zero if sufficiently close to 2*pi.
     if(std::abs(wrapped - 2*M_PI) < 1e-8){
       wrapped = 0;
     }

     // Ensure angle is positive.
     if(wrapped < 0){
        wrapped += 2*M_PI;
     }

     return wrapped;
}

void GazeboMotorModel::UpdateForcesAndMoments() {
  switch (motor_type_) {
    case (MotorType::kPosition): {
      double err = NormalizeAngle(joint_->Position(0)) - NormalizeAngle(ref_motor_input_);

      // Angles are element of [0..2pi).
      // Constrain difference of angles to be in [-pi..pi).
      if (err > M_PI){
        err -= 2*M_PI;
      }
      if (err < -M_PI){
        err += 2*M_PI;
      }
      if(std::abs(err - M_PI) < 1e-8){
        err = -M_PI;
      }

      double force = pids_.Update(err, sampling_time_);
      joint_->SetForce(0, force);
      break;
    }
    case (MotorType::kForce): {
      joint_->SetForce(0, ref_motor_input_);
      break;
    }
    default:  // MotorType::kVelocity
    {
      motor_rot_vel_ = joint_->GetVelocity(0);
      if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
        gzerr << "Aliasing on motor [" << motor_number_
              << "] might occur. Consider making smaller simulation time "
                 "steps or raising the rotor_velocity_slowdown_sim_ param.\n";
      }
      double real_motor_velocity =
          motor_rot_vel_ * rotor_velocity_slowdown_sim_;
      // Get the direction of the rotor rotation.
      int real_motor_velocity_sign =
          (real_motor_velocity > 0) - (real_motor_velocity < 0);
      // Assuming symmetric propellers (or rotors) for the thrust calculation.
      double thrust = turning_direction_ * real_motor_velocity_sign *
                      real_motor_velocity * real_motor_velocity *
                      motor_constant_;

      // Apply a force to the link.
      link_->AddRelativeForce(ignition::math::Vector3d (0, 0, thrust));

      // Forces from Philppe Martin's and Erwan Salaün's
      // 2010 IEEE Conference on Robotics and Automation paper
      // The True Role of Accelerometer Feedback in Quadrotor Control
      // - \omega * \lambda_1 * V_A^{\perp}
      ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
      ignition::math::Vector3d body_velocity_W = link_->WorldLinearVel();
      ignition::math::Vector3d relative_wind_velocity_W = body_velocity_W - wind_speed_W_;
      ignition::math::Vector3d body_velocity_perpendicular =
          relative_wind_velocity_W -
          (relative_wind_velocity_W.Dot(joint_axis) * joint_axis);
      ignition::math::Vector3d air_drag = -std::abs(real_motor_velocity) *
                               rotor_drag_coefficient_ *
                               body_velocity_perpendicular;

      // Apply air_drag to link.
      link_->AddForce(air_drag);

      /***********************************************
       * Start synthetic drag force model
       ***********************************************/
      /*
       * Synthetic configuration dependent force term.
       * The synthetic force is introduced to simulate a kind of body drag. It is based one the assumption that
       * body drag is significant and changes depending on the configuration. Here we use a very simple model
       * that assumes that the forces depend linearly on the linear velocities along body x and y axis in the
       * body frame scaled by a constant.
       * Ideally this model could be integrated or added as a plugin to foldable_drone_simulation pkg
       * see foldable_drone_simulation/src/foldable_drone_joints_controller.cpp
      */
      std::string joint_servo_name = "joint" + std::to_string(motor_number_);
      physics::JointPtr joint_servo = this->model_->GetJoint(joint_servo_name);
      if (joint_servo == NULL)
        gzthrow(
          "[gazebo_motor_model] Couldn't find specified joint \"" << joint_servo_name
                                                                  << "\".");
      // Get joint angle and limit it to the relevant range (here 0 to 90 for simplicity)
      const double servo_angle = joint_servo->Position(0);
      const double servo_angle_bounded = std::max(std::min(1.57, servo_angle), 0.0);

      // Init vectors
      ignition::math::Vector3d synthetic_drag_force_body(0.0, 0.0, 0.0);
      ignition::math::Vector3d scale_factor_body(0.0, 0.0, 0.0);

      // Debug input for testing (constant) => combined with servo_angle_trackbar to change servo angles, turn off AddRelativeForce()
      // const double _real_motor_velocity = 400.0;
      // ignition::math::Vector3d _body_velocity_perpendicular(1.0, 1.0, 0.0);

      // Transform  body_velocity_perpendicular from world in body frame
      ignition::math::Pose3d pose_link = base_link_->WorldCoGPose();
      ignition::math::Vector3d body_velocity_perpendicular_body = pose_link.Rot().RotateVectorReverse(body_velocity_perpendicular);

      // Actual synthetic force model (can be changed as needed, goal would be to also have a nonlinear one wrt to servo_angle_i,
      // it could also be extended to a wider range of angles)
      // drag_force_x = [servo_angle_0 + (1.57 - servo_angle_1) + servo_angle_2 + (1.57 - servo_angle_3)]
      //                  * kAvgMotorSpeed * rotor_drag_angle_coefficient_ * body_velocity_perpendicular_body_x
      // drag_force_y = [(1.57 - servo_angle_0) + servo_angle_1 + (1.57 - servo_angle_2) + servo_angle_3]
      //                  * kAvgMotorSpeed * rotor_drag_angle_coefficient_ * body_velocity_perpendicular_body_y
      // drag_force_z = 0

      const double kAvgMotorSpeed = 400.0;  // constant factor to be able to use coefficients of same order of magnitude as for rotor drag

      if(motor_number_ == 0 || motor_number_ == 2){
        scale_factor_body = ignition::math::Vector3d(servo_angle_bounded, 1.57 - servo_angle_bounded, 0.0);
        synthetic_drag_force_body = -kAvgMotorSpeed*rotor_drag_angle_coefficient_ *
                                scale_factor_body *
                                body_velocity_perpendicular_body;
      }
      if(motor_number_ == 1 || motor_number_ == 3){
        scale_factor_body = ignition::math::Vector3d(1.57 - servo_angle_bounded, servo_angle_bounded, 0.0);
        synthetic_drag_force_body  = -kAvgMotorSpeed*rotor_drag_angle_coefficient_ *
                                scale_factor_body *
                                body_velocity_perpendicular_body;
      }
      // Add force to base_link s.t. no torques are induced, only forces
      base_link_->AddRelativeForce(synthetic_drag_force_body);

      // Debug output for only one motor (activate gzdbg output by setting arg verbose=True in launch file)
      if(motor_number_ == 1) {
        gzdbg << "Orient: " << pose_link.Rot().Euler().X() << ", " << pose_link.Rot().Euler().Y() << ", "
              << pose_link.Rot().Euler().Z() << std::endl;
        gzdbg << "body vel: " << body_velocity_perpendicular.X() << ", " << body_velocity_perpendicular.Y() << ", "
              << body_velocity_perpendicular.Z() << std::endl;
        gzdbg << "body vel body reverse: " << body_velocity_perpendicular_body.X() << ", " << body_velocity_perpendicular_body.Y() << ", "
              << body_velocity_perpendicular_body.Z() << std::endl;
        gzdbg << "forces: " << std::to_string(synthetic_drag_force_body.X()) << ", " << std::to_string(synthetic_drag_force_body.Y()) << ", "
              << std::to_string(synthetic_drag_force_body.Z()) << std::endl;
      }

      // Debug output for all motors (activate gzdbg output by setting arg verbose=True in launch file)
      // gzdbg << motor_number_ << " has angle " << servo_angle_bounded << std::endl;
      // gzdbg << "coeff: " << std::to_string(rotor_drag_angle_coefficient_)   << std::endl;
      // gzdbg << "scale: " << scale_factor_body.X() << ", " << scale_factor_body.Y() << ", " << scale_factor_body.Z() << std::endl;
      // gzdbg << "scale: " << scale_factor_world.X() << ", " << scale_factor_world.Y() << ", " << scale_factor_world.Z() << std::endl;
      // gzdbg << "body vel: " << body_velocity_perpendicular.X() << ", " << body_velocity_perpendicular.Y() << ", " << body_velocity_perpendicular.Z() << std::endl;
      // gzdbg << "forces: " << std::to_string(synthetic_drag_force.X()) << ", " << std::to_string(synthetic_drag_force.Y()) << ", " << std::to_string(synthetic_drag_force.Z()) << std::endl;

      /*********************************************
       * End of synthetic drag force model
       *********************************************/

      // Moments get the parent link, such that the resulting torques can be
      // applied.
      physics::Link_V parent_links = link_->GetParentJointsLinks();
      // The tansformation from the parent_link to the link_.
      ignition::math::Pose3d pose_difference =
          link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
      ignition::math::Vector3d drag_torque(
          0, 0, -turning_direction_ * thrust * moment_constant_);
      // Transforming the drag torque into the parent frame to handle
      // arbitrary rotor orientations.
      ignition::math::Vector3d drag_torque_parent_frame =
          pose_difference.Rot().RotateVector(drag_torque);
      parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

      ignition::math::Vector3d rolling_moment;
      // - \omega * \mu_1 * V_A^{\perp}
      rolling_moment = -std::abs(real_motor_velocity) *
                       rolling_moment_coefficient_ *
                       body_velocity_perpendicular;
      parent_links.at(0)->AddTorque(rolling_moment);
      // Apply the filter on the motor's velocity.
      double ref_motor_rot_vel;
      ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(
          ref_motor_input_, sampling_time_);

      // Make sure max force is set, as it may be reset to 0 by a world reset any
      // time. (This cannot be done during Reset() because the change will be undone
      // by the Joint's reset function afterwards.)
      joint_->SetVelocity(
          0, turning_direction_ * ref_motor_rot_vel /
                 rotor_velocity_slowdown_sim_);
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel);
}
