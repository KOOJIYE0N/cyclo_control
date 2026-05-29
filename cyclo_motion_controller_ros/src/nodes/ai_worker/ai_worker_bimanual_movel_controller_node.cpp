#include "cyclo_motion_controller_ros/nodes/ai_worker/ai_worker_bimanual_movel_controller_node.hpp"

#include <algorithm>

namespace cyclo_motion_controller_ros
{
AIWorkerBimanualMoveLControllerNode::AIWorkerBimanualMoveLControllerNode()
: Node("ai_worker_bimanual_movel_controller"),
  last_joint_state_time_(this->now()),
  last_right_goal_cmd_time_(this->now()),
  last_left_goal_cmd_time_(this->now())
{
  control_frequency_ = this->declare_parameter("control_frequency", 100.0);
  time_step_ = this->declare_parameter("time_step", 0.01);
  trajectory_time_ = this->declare_parameter("trajectory_time", 0.0);
  kp_position_ = this->declare_parameter("kp_position", 50.0);
  kp_orientation_ = this->declare_parameter("kp_orientation", 50.0);
  weight_position_ = this->declare_parameter("weight_position", 10.0);
  weight_orientation_ = this->declare_parameter("weight_orientation", 1.0);
  weight_damping_ = this->declare_parameter("weight_damping", 0.1);
  slack_penalty_ = this->declare_parameter("slack_penalty", 1000.0);
  cbf_alpha_ = this->declare_parameter("cbf_alpha", 5.0);
  collision_buffer_ = this->declare_parameter("collision_buffer", 0.05);
  collision_safe_distance_ = this->declare_parameter("collision_safe_distance", 0.02);
  rigid_grasp_position_recovery_gain_ =
    this->declare_parameter("rigid_grasp_position_recovery_gain", 10.0);
  rigid_grasp_orientation_recovery_gain_ =
    this->declare_parameter("rigid_grasp_orientation_recovery_gain", 10.0);
  joint_state_timeout_ = this->declare_parameter("joint_state_timeout", 0.5);
  goal_command_timeout_ = this->declare_parameter("goal_command_timeout", 0.2);
  passive_hold_weight_scale_ = this->declare_parameter("passive_hold_weight_scale", 5.0);
  grasp_blend_ratio_ = this->declare_parameter("grasp_blend_ratio", 0.5);
  urdf_path_ = this->declare_parameter("urdf_path", std::string(""));
  srdf_path_ = this->declare_parameter("srdf_path", std::string(""));
  joint_states_topic_ = this->declare_parameter("joint_states_topic", std::string("/joint_states"));
  right_goal_pose_topic_ = this->declare_parameter("right_goal_pose_topic", std::string("/r_goal_pose"));
  left_goal_pose_topic_ = this->declare_parameter("left_goal_pose_topic", std::string("/l_goal_pose"));
  virtual_object_pose_topic_ = this->declare_parameter(
    "virtual_object_pose_topic", std::string("/virtual_object_goal_pose"));
  grasp_capture_topic_ = this->declare_parameter("grasp_capture_topic", std::string("/capture_grasp"));
  right_traj_topic_ = this->declare_parameter(
    "right_traj_topic",
    std::string("/leader/joint_trajectory_command_broadcaster_right/joint_trajectory"));
  left_traj_topic_ = this->declare_parameter(
    "left_traj_topic",
    std::string("/leader/joint_trajectory_command_broadcaster_left/joint_trajectory"));
  lift_topic_ = this->declare_parameter(
    "lift_topic", std::string("/leader/joystick_controller_right/joint_trajectory"));
  lift_vel_bound_ = this->declare_parameter("lift_vel_bound", 0.0);
  r_gripper_pose_topic_ = this->declare_parameter("r_gripper_pose_topic", std::string("/r_gripper_pose"));
  l_gripper_pose_topic_ = this->declare_parameter("l_gripper_pose_topic", std::string("/l_gripper_pose"));
  r_gripper_name_ = this->declare_parameter("r_gripper_name", std::string("arm_r_link7"));
  l_gripper_name_ = this->declare_parameter("l_gripper_name", std::string("arm_l_link7"));

  if (urdf_path_.empty()) {
    RCLCPP_FATAL(this->get_logger(), "URDF path not provided.");
    rclcpp::shutdown();
    return;
  }

  right_goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    right_goal_pose_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&AIWorkerBimanualMoveLControllerNode::rightGoalPoseCallback, this, std::placeholders::_1));
  left_goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    left_goal_pose_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&AIWorkerBimanualMoveLControllerNode::leftGoalPoseCallback, this, std::placeholders::_1));
  virtual_object_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    virtual_object_pose_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&AIWorkerBimanualMoveLControllerNode::virtualObjectPoseCallback, this, std::placeholders::_1));
  grasp_capture_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    grasp_capture_topic_, 10,
    std::bind(&AIWorkerBimanualMoveLControllerNode::graspCaptureCallback, this, std::placeholders::_1));
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, 10,
    std::bind(&AIWorkerBimanualMoveLControllerNode::jointStateCallback, this, std::placeholders::_1));

  arm_r_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(right_traj_topic_, 10);
  arm_l_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(left_traj_topic_, 10);
  lift_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(lift_topic_, 10);
  r_gripper_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(r_gripper_pose_topic_, 10);
  l_gripper_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(l_gripper_pose_topic_, 10);

  try {
    kinematics_solver_ = std::make_shared<cyclo_motion_controller::kinematics::KinematicsSolver>(
      urdf_path_, srdf_path_);
    qp_controller_ = std::make_shared<cyclo_motion_controller::controllers::AIWorkerBimanualMoveLController>(
      kinematics_solver_, time_step_);
    qp_controller_->setControllerParams(
      slack_penalty_, cbf_alpha_, collision_buffer_, collision_safe_distance_);
    qp_controller_->setConstraintLinks(r_gripper_name_, l_gripper_name_);

    const int dof = kinematics_solver_->getDof();
    q_.setZero(dof);
    qdot_.setZero(dof);
    q_desired_.setZero(dof);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize bimanual controller: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  initializeJointConfig();

  const int timer_period_ms =
    std::max(1, static_cast<int>(std::round(1000.0 / std::max(1.0, control_frequency_))));
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_period_ms),
    std::bind(&AIWorkerBimanualMoveLControllerNode::controlLoopCallback, this));
}

AIWorkerBimanualMoveLControllerNode::~AIWorkerBimanualMoveLControllerNode() {}

void AIWorkerBimanualMoveLControllerNode::initializeJointConfig()
{
  model_joint_names_ = kinematics_solver_->getJointNames();
  model_joint_index_map_.clear();
  for (size_t i = 0; i < model_joint_names_.size(); ++i) {
    model_joint_index_map_[model_joint_names_[i]] = static_cast<int>(i);
  }

  left_arm_joints_.clear();
  right_arm_joints_.clear();
  lift_joint_.clear();
  lift_joint_index_ = -1;
  for (const auto & joint_name : model_joint_names_) {
    if (joint_name.find("arm_l_joint") != std::string::npos) {
      left_arm_joints_.push_back(joint_name);
    } else if (joint_name.find("arm_r_joint") != std::string::npos) {
      right_arm_joints_.push_back(joint_name);
    } else if (joint_name.find("lift_joint") != std::string::npos) {
      lift_joint_ = joint_name;
    }
  }
  std::sort(left_arm_joints_.begin(), left_arm_joints_.end());
  std::sort(right_arm_joints_.begin(), right_arm_joints_.end());

  if (!lift_joint_.empty()) {
    auto lift_it = model_joint_index_map_.find(lift_joint_);
    if (lift_it != model_joint_index_map_.end()) {
      lift_joint_index_ = lift_it->second;
      const bool locked = kinematics_solver_->setJointVelocityBoundsByIndex(
        lift_joint_index_, -lift_vel_bound_, lift_vel_bound_);
      if (!locked) {
        lift_joint_index_ = -1;
      }
    }
  }
}

void AIWorkerBimanualMoveLControllerNode::extractJointStates(
  const sensor_msgs::msg::JointState::SharedPtr & msg)
{
  const int dof = kinematics_solver_->getDof();
  q_.setZero(dof);
  qdot_.setZero(dof);

  const int max_index = std::min<int>(dof, static_cast<int>(model_joint_names_.size()));
  for (int i = 0; i < max_index; ++i) {
    const auto & joint_name = model_joint_names_[i];
    auto it = joint_index_map_.find(joint_name);
    if (it == joint_index_map_.end()) {
      continue;
    }
    const int idx = it->second;
    if (idx < static_cast<int>(msg->position.size())) {
      q_[i] = msg->position[idx];
    }
    if (idx < static_cast<int>(msg->velocity.size())) {
      qdot_[i] = msg->velocity[idx];
    }
  }
}

void AIWorkerBimanualMoveLControllerNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (joint_index_map_.empty()) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_index_map_[msg->name[i]] = static_cast<int>(i);
    }
  }

  extractJointStates(msg);
  last_joint_state_time_ = this->now();
  joint_state_received_ = true;
  joint_state_timeout_active_ = false;

  if (!q_desired_initialized_) {
    syncCommandStateToFeedback();
    q_desired_initialized_ = true;
  }
}

void AIWorkerBimanualMoveLControllerNode::rightGoalPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  right_goal_pose_ = poseMsgToEigen(*msg);
  right_goal_pose_received_ = true;
  right_goal_pose_updated_ = true;
  last_right_goal_cmd_time_ = this->now();
}

void AIWorkerBimanualMoveLControllerNode::leftGoalPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  left_goal_pose_ = poseMsgToEigen(*msg);
  left_goal_pose_received_ = true;
  left_goal_pose_updated_ = true;
  last_left_goal_cmd_time_ = this->now();
}

void AIWorkerBimanualMoveLControllerNode::virtualObjectPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  virtual_object_goal_pose_ = poseMsgToEigen(*msg);
  virtual_object_goal_received_ = true;
  virtual_object_goal_updated_ = true;
}

void AIWorkerBimanualMoveLControllerNode::graspCaptureCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (!msg->data) {
    grasp_constraint_active_ = false;
    qp_controller_->setRigidGraspPoseConstraint(false, Eigen::Affine3d::Identity());
    if (joint_state_received_) {
      Eigen::VectorXd q_feedback = q_desired_initialized_ ? q_desired_ : q_;
      if (lift_joint_index_ >= 0 && lift_joint_index_ < q_feedback.size()) {
        q_feedback[lift_joint_index_] = q_[lift_joint_index_];
      }
      kinematics_solver_->updateState(q_feedback, qdot_);
      right_goal_pose_ = kinematics_solver_->getPose(r_gripper_name_);
      left_goal_pose_ = kinematics_solver_->getPose(l_gripper_name_);
      right_goal_pose_received_ = true;
      left_goal_pose_received_ = true;
      right_goal_pose_updated_ = false;
      left_goal_pose_updated_ = false;
    }
    virtual_object_goal_updated_ = false;
    return;
  }
  captureCurrentGraspConstraint();
}

Eigen::Affine3d AIWorkerBimanualMoveLControllerNode::poseMsgToEigen(
  const geometry_msgs::msg::PoseStamped & pose_msg) const
{
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translation() << pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z;
  const Eigen::Quaterniond quat(
    pose_msg.pose.orientation.w,
    pose_msg.pose.orientation.x,
    pose_msg.pose.orientation.y,
    pose_msg.pose.orientation.z);
  pose.linear() = quat.normalized().toRotationMatrix();
  return pose;
}

cyclo_motion_controller::common::Vector6d AIWorkerBimanualMoveLControllerNode::computeDesiredVelocity(
  const Eigen::Affine3d & current_pose,
  const Eigen::Affine3d & goal_pose) const
{
  cyclo_motion_controller::common::Vector6d desired_vel =
    cyclo_motion_controller::common::Vector6d::Zero();
  const Eigen::Vector3d position_error = goal_pose.translation() - current_pose.translation();
  const Eigen::Matrix3d rotation_error = goal_pose.linear() * current_pose.linear().transpose();
  const Eigen::AngleAxisd angle_axis_error(rotation_error);
  const Eigen::Vector3d orientation_error = angle_axis_error.axis() * angle_axis_error.angle();
  desired_vel.head<3>() = kp_position_ * position_error;
  desired_vel.tail<3>() = kp_orientation_ * orientation_error;
  return desired_vel;
}

Eigen::Affine3d AIWorkerBimanualMoveLControllerNode::blendPoses(
  const Eigen::Affine3d & pose_a,
  const Eigen::Affine3d & pose_b,
  const double blend) const
{
  const double alpha = std::clamp(blend, 0.0, 1.0);
  const Eigen::Vector3d blended_position =
    (1.0 - alpha) * pose_a.translation() + alpha * pose_b.translation();
  const Eigen::Quaterniond qa(pose_a.linear());
  const Eigen::Quaterniond qb(pose_b.linear());
  const Eigen::Quaterniond blended_orientation = qa.slerp(alpha, qb).normalized();

  Eigen::Affine3d blended_pose = Eigen::Affine3d::Identity();
  blended_pose.translation() = blended_position;
  blended_pose.linear() = blended_orientation.toRotationMatrix();
  return blended_pose;
}

void AIWorkerBimanualMoveLControllerNode::captureCurrentGraspConstraint()
{
  if (!joint_state_received_) {
    return;
  }
  const Eigen::VectorXd q_feedback = q_desired_initialized_ ? q_desired_ : q_;
  kinematics_solver_->updateState(q_feedback, qdot_);
  const Eigen::Affine3d right_pose = kinematics_solver_->getPose(r_gripper_name_);
  const Eigen::Affine3d left_pose = kinematics_solver_->getPose(l_gripper_name_);
  Eigen::Affine3d object_pose = Eigen::Affine3d::Identity();
  object_pose.translation() = 0.5 * (right_pose.translation() + left_pose.translation());
  const Eigen::Quaterniond qr(right_pose.linear());
  const Eigen::Quaterniond ql(left_pose.linear());
  object_pose.linear() = qr.slerp(0.5, ql).normalized().toRotationMatrix();
  grasp_object_to_right_ = object_pose.inverse() * right_pose;
  grasp_object_to_left_ = object_pose.inverse() * left_pose;
  grasp_constraint_active_ = true;
  virtual_object_goal_pose_ = object_pose;
  virtual_object_goal_received_ = true;
  virtual_object_goal_updated_ = false;
}

void AIWorkerBimanualMoveLControllerNode::applyBimanualGoalProjection(
  Eigen::Affine3d & right_goal_pose,
  Eigen::Affine3d & left_goal_pose) const
{
  if (!grasp_constraint_active_) {
    return;
  }

  const Eigen::Affine3d object_from_right = right_goal_pose * grasp_object_to_right_.inverse();
  const Eigen::Affine3d object_from_left = left_goal_pose * grasp_object_to_left_.inverse();
  const Eigen::Affine3d blended_object_pose =
    blendPoses(object_from_right, object_from_left, grasp_blend_ratio_);

  right_goal_pose = blended_object_pose * grasp_object_to_right_;
  left_goal_pose = blended_object_pose * grasp_object_to_left_;
}

void AIWorkerBimanualMoveLControllerNode::controlLoopCallback()
{
  if (!joint_state_received_ || !q_desired_initialized_) {
    return;
  }
  if (jointStateTimedOut()) {
    joint_state_timeout_active_ = true;
    return;
  }

  try {
    Eigen::VectorXd q_feedback = q_desired_;
    if (lift_joint_index_ >= 0 && lift_joint_index_ < q_feedback.size()) {
      q_feedback[lift_joint_index_] = q_[lift_joint_index_];
    }
    kinematics_solver_->updateState(q_feedback, qdot_);
    right_gripper_pose_ = kinematics_solver_->getPose(r_gripper_name_);
    left_gripper_pose_ = kinematics_solver_->getPose(l_gripper_name_);
    publishGripperPose(right_gripper_pose_, left_gripper_pose_);

    if (!right_goal_pose_received_) {
      right_goal_pose_ = right_gripper_pose_;
    }
    if (!left_goal_pose_received_) {
      left_goal_pose_ = left_gripper_pose_;
    }

    Eigen::Affine3d constrained_right_goal = right_goal_pose_;
    Eigen::Affine3d constrained_left_goal = left_goal_pose_;
    const bool right_cmd_active =
      (this->now() - last_right_goal_cmd_time_).seconds() <= goal_command_timeout_;
    const bool left_cmd_active =
      (this->now() - last_left_goal_cmd_time_).seconds() <= goal_command_timeout_;
    if (!grasp_constraint_active_) {
      if (right_cmd_active && !left_cmd_active) {
        constrained_left_goal = left_gripper_pose_;
        left_goal_pose_ = left_gripper_pose_;
      } else if (left_cmd_active && !right_cmd_active) {
        constrained_right_goal = right_gripper_pose_;
        right_goal_pose_ = right_gripper_pose_;
      } else if (!right_cmd_active && !left_cmd_active) {
        constrained_right_goal = right_gripper_pose_;
        constrained_left_goal = left_gripper_pose_;
        right_goal_pose_ = right_gripper_pose_;
        left_goal_pose_ = left_gripper_pose_;
      }
    } else {
      Eigen::Affine3d object_goal = right_gripper_pose_;
      if (virtual_object_goal_received_) {
        object_goal = virtual_object_goal_pose_;
      } else {
        virtual_object_goal_pose_ = right_gripper_pose_;
        virtual_object_goal_received_ = true;
      }
      constrained_right_goal = object_goal * grasp_object_to_right_;
      constrained_left_goal = object_goal * grasp_object_to_left_;
      right_goal_pose_ = constrained_right_goal;
      left_goal_pose_ = constrained_left_goal;
      virtual_object_goal_updated_ = false;
    }
    right_goal_pose_updated_ = false;
    left_goal_pose_updated_ = false;

    std::map<std::string, cyclo_motion_controller::common::Vector6d> desired_task_velocities;
    desired_task_velocities[r_gripper_name_] =
      computeDesiredVelocity(right_gripper_pose_, constrained_right_goal);
    desired_task_velocities[l_gripper_name_] =
      computeDesiredVelocity(left_gripper_pose_, constrained_left_goal);

    std::map<std::string, cyclo_motion_controller::common::Vector6d> weights;
    cyclo_motion_controller::common::Vector6d right_weight =
      cyclo_motion_controller::common::Vector6d::Ones();
    cyclo_motion_controller::common::Vector6d left_weight =
      cyclo_motion_controller::common::Vector6d::Ones();
    right_weight.head<3>().setConstant(weight_position_);
    right_weight.tail<3>().setConstant(weight_orientation_);
    left_weight.head<3>().setConstant(weight_position_);
    left_weight.tail<3>().setConstant(weight_orientation_);
    if (!grasp_constraint_active_) {
      if (right_cmd_active && !left_cmd_active) {
        left_weight.head<3>().setConstant(weight_position_ * passive_hold_weight_scale_);
        left_weight.tail<3>().setConstant(weight_orientation_ * passive_hold_weight_scale_);
      } else if (left_cmd_active && !right_cmd_active) {
        right_weight.head<3>().setConstant(weight_position_ * passive_hold_weight_scale_);
        right_weight.tail<3>().setConstant(weight_orientation_ * passive_hold_weight_scale_);
      }
    }
    weights[r_gripper_name_] = right_weight;
    weights[l_gripper_name_] = left_weight;

    if (grasp_constraint_active_) {
      const Eigen::Affine3d right_to_left_transform =
        grasp_object_to_right_.inverse() * grasp_object_to_left_;
      qp_controller_->setRigidGraspPoseConstraint(
        true,
        right_to_left_transform);
    } else {
      qp_controller_->setRigidGraspPoseConstraint(false, Eigen::Affine3d::Identity());
    }

    const Eigen::VectorXd damping =
      Eigen::VectorXd::Ones(kinematics_solver_->getDof()) * weight_damping_;
    qp_controller_->setWeight(weights, damping);
    qp_controller_->setDesiredTaskVel(desired_task_velocities);

    Eigen::VectorXd optimal_velocities;
    if (!qp_controller_->getOptJointVel(optimal_velocities)) {
      return;
    }

    q_desired_ = q_feedback + optimal_velocities * time_step_;
    publishTrajectory(q_desired_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Bimanual controller loop error: %s", e.what());
  }
}

bool AIWorkerBimanualMoveLControllerNode::jointStateTimedOut() const
{
  return joint_state_received_ &&
         (this->now() - last_joint_state_time_).seconds() > joint_state_timeout_;
}

void AIWorkerBimanualMoveLControllerNode::syncCommandStateToFeedback()
{
  q_desired_ = q_;
}

void AIWorkerBimanualMoveLControllerNode::syncArmStateToFeedback(
  const std::vector<std::string> & arm_joint_names,
  Eigen::VectorXd & destination) const
{
  for (const auto & joint_name : arm_joint_names) {
    const auto it = model_joint_index_map_.find(joint_name);
    if (it == model_joint_index_map_.end()) {
      continue;
    }
    destination[it->second] = q_[it->second];
  }
}

void AIWorkerBimanualMoveLControllerNode::publishTrajectory(const Eigen::VectorXd & q_desired) const
{
  std::vector<int> left_arm_indices;
  std::vector<int> right_arm_indices;

  for (const auto & joint_name : left_arm_joints_) {
    auto it = model_joint_index_map_.find(joint_name);
    if (it != model_joint_index_map_.end()) {
      left_arm_indices.push_back(it->second);
    }
  }
  for (const auto & joint_name : right_arm_joints_) {
    auto it = model_joint_index_map_.find(joint_name);
    if (it != model_joint_index_map_.end()) {
      right_arm_indices.push_back(it->second);
    }
  }

  if (!left_arm_indices.empty()) {
    arm_l_pub_->publish(createArmTrajectoryMsg(left_arm_joints_, q_desired, left_arm_indices));
  }
  if (!right_arm_indices.empty()) {
    arm_r_pub_->publish(createArmTrajectoryMsg(right_arm_joints_, q_desired, right_arm_indices));
  }
  if (lift_joint_index_ >= 0 && !lift_joint_.empty() && lift_vel_bound_ != 0.0 &&
    lift_joint_index_ < q_desired.size())
  {
    lift_pub_->publish(createLiftTrajectoryMsg(lift_joint_, q_desired[lift_joint_index_]));
  }
}

trajectory_msgs::msg::JointTrajectory AIWorkerBimanualMoveLControllerNode::createArmTrajectoryMsg(
  const std::vector<std::string> & arm_joint_names,
  const Eigen::VectorXd & positions,
  const std::vector<int> & arm_indices) const
{
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.header.frame_id = "";
  traj_msg.joint_names = arm_joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(trajectory_time_);
  for (int idx : arm_indices) {
    if (idx >= 0 && idx < static_cast<int>(positions.size())) {
      point.positions.push_back(positions[idx]);
    }
  }
  traj_msg.points.push_back(point);
  return traj_msg;
}

trajectory_msgs::msg::JointTrajectory AIWorkerBimanualMoveLControllerNode::createLiftTrajectoryMsg(
  const std::string & lift_joint_name, const double position) const
{
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.header.frame_id = "";
  traj_msg.joint_names = {lift_joint_name};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(trajectory_time_);
  point.positions = {position};
  traj_msg.points.push_back(point);
  return traj_msg;
}

void AIWorkerBimanualMoveLControllerNode::publishGripperPose(
  const Eigen::Affine3d & right_pose,
  const Eigen::Affine3d & left_pose)
{
  geometry_msgs::msg::PoseStamped right_msg;
  right_msg.header.stamp = this->now();
  right_msg.header.frame_id = "base_link";
  right_msg.pose.position.x = right_pose.translation().x();
  right_msg.pose.position.y = right_pose.translation().y();
  right_msg.pose.position.z = right_pose.translation().z();
  const Eigen::Quaterniond right_quat(right_pose.linear());
  right_msg.pose.orientation.w = right_quat.w();
  right_msg.pose.orientation.x = right_quat.x();
  right_msg.pose.orientation.y = right_quat.y();
  right_msg.pose.orientation.z = right_quat.z();
  r_gripper_pose_pub_->publish(right_msg);

  geometry_msgs::msg::PoseStamped left_msg;
  left_msg.header.stamp = this->now();
  left_msg.header.frame_id = "base_link";
  left_msg.pose.position.x = left_pose.translation().x();
  left_msg.pose.position.y = left_pose.translation().y();
  left_msg.pose.position.z = left_pose.translation().z();
  const Eigen::Quaterniond left_quat(left_pose.linear());
  left_msg.pose.orientation.w = left_quat.w();
  left_msg.pose.orientation.x = left_quat.x();
  left_msg.pose.orientation.y = left_quat.y();
  left_msg.pose.orientation.z = left_quat.z();
  l_gripper_pose_pub_->publish(left_msg);
}

}  // namespace cyclo_motion_controller_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cyclo_motion_controller_ros::AIWorkerBimanualMoveLControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
