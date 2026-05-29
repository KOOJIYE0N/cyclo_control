#include "cyclo_motion_controller_ros/nodes/ai_worker/ai_worker_bimanual_movej_controller_node.hpp"

#include <algorithm>

namespace cyclo_motion_controller_ros
{
AIWorkerBimanualMoveJController::AIWorkerBimanualMoveJController()
: Node("ai_worker_bimanual_movej_controller"),
  joint_state_received_(false),
  commanded_state_initialized_(false),
  right_movej_target_initialized_(false),
  left_movej_target_initialized_(false),
  right_gripper_position_(0.0),
  left_gripper_position_(0.0),
  last_joint_state_time_(this->now())
{
  control_frequency_ = this->declare_parameter("control_frequency", 100.0);
  time_step_ = this->declare_parameter("time_step", 0.01);
  trajectory_time_ = this->declare_parameter("trajectory_time", 0.0);
  kp_joint_ = this->declare_parameter("kp_joint", 6.0);
  weight_tracking_ = this->declare_parameter("weight_tracking", 1.0);
  kp_grasp_position_ = this->declare_parameter("kp_grasp_position", 10.0);
  kp_grasp_orientation_ = this->declare_parameter("kp_grasp_orientation", 10.0);
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
  gripper_grasp_threshold_ = this->declare_parameter("gripper_grasp_threshold", 0.85);
  gripper_grasp_hold_time_ = this->declare_parameter("gripper_grasp_hold_time", 2.0);
  urdf_path_ = this->declare_parameter("urdf_path", std::string(""));
  srdf_path_ = this->declare_parameter("srdf_path", std::string(""));
  joint_states_topic_ = this->declare_parameter("joint_states_topic", std::string("/joint_states"));
  right_traj_topic_ = this->declare_parameter(
    "right_traj_topic",
    std::string("/leader/joint_trajectory_command_broadcaster_right/raw_joint_trajectory"));
  left_traj_topic_ = this->declare_parameter(
    "left_traj_topic",
    std::string("/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory"));
  grasp_capture_topic_ = this->declare_parameter("grasp_capture_topic", std::string("/capture_grasp"));
  right_traj_filtered_topic_ = this->declare_parameter(
    "right_traj_filtered_topic",
    std::string("/leader/joint_trajectory_command_broadcaster_right/joint_trajectory"));
  left_traj_filtered_topic_ = this->declare_parameter(
    "left_traj_filtered_topic",
    std::string("/leader/joint_trajectory_command_broadcaster_left/joint_trajectory"));
  right_gripper_joint_name_ = this->declare_parameter(
    "right_gripper_joint", std::string("gripper_r_joint1"));
  left_gripper_joint_name_ = this->declare_parameter(
    "left_gripper_joint", std::string("gripper_l_joint1"));
  r_gripper_name_ = this->declare_parameter("r_gripper_name", std::string("arm_r_link7"));
  l_gripper_name_ = this->declare_parameter("l_gripper_name", std::string("arm_l_link7"));
  grasp_blend_ratio_ = this->declare_parameter("grasp_blend_ratio", 0.5);

  if (urdf_path_.empty()) {
    RCLCPP_FATAL(this->get_logger(), "URDF path not provided.");
    rclcpp::shutdown();
    return;
  }

  arm_r_pub_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>(right_traj_filtered_topic_, 10);
  arm_l_pub_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>(left_traj_filtered_topic_, 10);
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, 10,
    std::bind(&AIWorkerBimanualMoveJController::jointStateCallback, this, std::placeholders::_1));
  r_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    right_traj_topic_, 10,
    std::bind(&AIWorkerBimanualMoveJController::rightTrajectoryCallback, this, std::placeholders::_1));
  l_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    left_traj_topic_, 10,
    std::bind(&AIWorkerBimanualMoveJController::leftTrajectoryCallback, this, std::placeholders::_1));
  grasp_capture_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    grasp_capture_topic_, 10,
    std::bind(&AIWorkerBimanualMoveJController::graspCaptureCallback, this, std::placeholders::_1));

  try {
    kinematics_solver_ =
      std::make_shared<cyclo_motion_controller::kinematics::KinematicsSolver>(urdf_path_, srdf_path_);
    qp_filter_ = std::make_shared<cyclo_motion_controller::controllers::AIWorkerBimanualMoveJController>(
      kinematics_solver_, time_step_);
    qp_filter_->setControllerParams(
      slack_penalty_, cbf_alpha_, collision_buffer_, collision_safe_distance_);
    qp_filter_->setConstraintLinks(r_gripper_name_, l_gripper_name_);

    const int dof = kinematics_solver_->getDof();
    q_.setZero(dof);
    qdot_.setZero(dof);
    q_commanded_.setZero(dof);
    right_movej_goal_.setZero(dof);
    left_movej_goal_.setZero(dof);
    initializeJointConfig();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize bimanual moveJ filter: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  const int timer_period_ms =
    std::max(1, static_cast<int>(std::round(1000.0 / std::max(1.0, control_frequency_))));
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_period_ms),
    std::bind(&AIWorkerBimanualMoveJController::controlLoopCallback, this));
}

AIWorkerBimanualMoveJController::~AIWorkerBimanualMoveJController() {}

void AIWorkerBimanualMoveJController::initializeJointConfig()
{
  model_joint_names_ = kinematics_solver_->getJointNames();
  model_joint_index_map_.clear();
  for (size_t i = 0; i < model_joint_names_.size(); ++i) {
    model_joint_index_map_[model_joint_names_[i]] = static_cast<int>(i);
  }

  left_arm_joints_.clear();
  right_arm_joints_.clear();
  for (const auto & joint_name : model_joint_names_) {
    if (joint_name.find("arm_l_joint") != std::string::npos) {
      left_arm_joints_.push_back(joint_name);
    } else if (joint_name.find("arm_r_joint") != std::string::npos) {
      right_arm_joints_.push_back(joint_name);
    }
  }

  std::sort(left_arm_joints_.begin(), left_arm_joints_.end());
  std::sort(right_arm_joints_.begin(), right_arm_joints_.end());
}

void AIWorkerBimanualMoveJController::extractJointStates(
  const sensor_msgs::msg::JointState::SharedPtr & msg)
{
  const int dof = kinematics_solver_->getDof();
  q_.setZero(dof);
  qdot_.setZero(dof);

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == right_gripper_joint_name_ && i < msg->position.size()) {
      right_gripper_joint_state_position_ = msg->position[i];
      right_gripper_joint_state_received_ = true;
    }
    if (msg->name[i] == left_gripper_joint_name_ && i < msg->position.size()) {
      left_gripper_joint_state_position_ = msg->position[i];
      left_gripper_joint_state_received_ = true;
    }
  }

  const int max_index = std::min<int>(dof, static_cast<int>(model_joint_names_.size()));
  for (int i = 0; i < max_index; ++i) {
    const auto & joint_name = model_joint_names_[i];
    const auto it = joint_index_map_.find(joint_name);
    if (it == joint_index_map_.end()) {
      continue;
    }
    const int msg_idx = it->second;
    if (msg_idx < static_cast<int>(msg->position.size())) {
      q_[i] = msg->position[msg_idx];
    }
    if (msg_idx < static_cast<int>(msg->velocity.size())) {
      qdot_[i] = msg->velocity[msg_idx];
    }
  }
}

void AIWorkerBimanualMoveJController::jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
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
  updateGripperTriggeredGraspMode();

  if (!commanded_state_initialized_) {
    syncCommandStateToFeedback();
    commanded_state_initialized_ = true;
    right_movej_target_initialized_ = true;
    left_movej_target_initialized_ = true;
  }
}

void AIWorkerBimanualMoveJController::updateGripperPositionFromTrajectory(
  const trajectory_msgs::msg::JointTrajectory & msg,
  const std::string & gripper_joint_name,
  double & gripper_position) const
{
  if (msg.points.empty() || msg.points.front().positions.empty()) {
    return;
  }
  const auto & point = msg.points.front();
  for (size_t i = 0; i < msg.joint_names.size(); ++i) {
    if (msg.joint_names[i] != gripper_joint_name) {
      continue;
    }
    if (i < point.positions.size()) {
      gripper_position = point.positions[i];
    }
    return;
  }
}

bool AIWorkerBimanualMoveJController::updateArmTargetFromTrajectory(
  const trajectory_msgs::msg::JointTrajectory & msg,
  const std::vector<std::string> & arm_joint_names,
  const std::string & arm_name,
  Eigen::VectorXd & target_q) const
{
  if (msg.points.empty()) {
    return false;
  }
  const auto & point = msg.points.front();
  if (point.positions.empty()) {
    RCLCPP_WARN(this->get_logger(), "%s bimanual moveJ ignored: positions are empty.", arm_name.c_str());
    return false;
  }

  if (!msg.joint_names.empty()) {
    for (size_t i = 0; i < msg.joint_names.size(); ++i) {
      if (i >= point.positions.size()) {
        continue;
      }
      const auto it = model_joint_index_map_.find(msg.joint_names[i]);
      if (it == model_joint_index_map_.end()) {
        continue;
      }
      target_q[it->second] = point.positions[i];
    }
    return true;
  }

  if (point.positions.size() == arm_joint_names.size()) {
    for (size_t i = 0; i < arm_joint_names.size(); ++i) {
      const auto model_it = model_joint_index_map_.find(arm_joint_names[i]);
      if (model_it == model_joint_index_map_.end()) {
        continue;
      }
      target_q[model_it->second] = point.positions[i];
    }
    return true;
  }

  RCLCPP_WARN(
    this->get_logger(),
    "%s bimanual moveJ ignored: joint_names missing and positions size does not match arm joints.",
    arm_name.c_str());
  return false;
}

void AIWorkerBimanualMoveJController::rightTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!msg || !joint_state_received_ || jointStateTimedOut() || !commanded_state_initialized_ || msg->points.empty()) {
    return;
  }
  if (rclcpp::Duration(msg->points.front().time_from_start).seconds() > 0.0) {
    syncRightArmToFeedback();
  }
  Eigen::VectorXd target_q = q_commanded_;
  if (!updateArmTargetFromTrajectory(*msg, right_arm_joints_, "Right", target_q)) {
    return;
  }
  updateGripperPositionFromTrajectory(*msg, right_gripper_joint_name_, right_gripper_position_);
  right_movej_goal_ = target_q;
  right_movej_target_initialized_ = true;
}

void AIWorkerBimanualMoveJController::leftTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!msg || !joint_state_received_ || jointStateTimedOut() || !commanded_state_initialized_ || msg->points.empty()) {
    return;
  }
  if (rclcpp::Duration(msg->points.front().time_from_start).seconds() > 0.0) {
    syncLeftArmToFeedback();
  }
  Eigen::VectorXd target_q = q_commanded_;
  if (!updateArmTargetFromTrajectory(*msg, left_arm_joints_, "Left", target_q)) {
    return;
  }
  updateGripperPositionFromTrajectory(*msg, left_gripper_joint_name_, left_gripper_position_);
  left_movej_goal_ = target_q;
  left_movej_target_initialized_ = true;
}

void AIWorkerBimanualMoveJController::graspCaptureCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (!msg->data) {
    disableGraspConstraint();
    return;
  }
  enableGraspConstraint();
}

void AIWorkerBimanualMoveJController::updateGripperTriggeredGraspMode()
{
  if (!right_gripper_joint_state_received_ || !left_gripper_joint_state_received_) {
    return;
  }

  const rclcpp::Time now = this->now();
  const bool both_closed =
    right_gripper_joint_state_position_ > gripper_grasp_threshold_ &&
    left_gripper_joint_state_position_ > gripper_grasp_threshold_;
  const bool both_open =
    right_gripper_joint_state_position_ < gripper_grasp_threshold_ &&
    left_gripper_joint_state_position_ < gripper_grasp_threshold_;

  if (!grasp_constraint_active_ && both_closed) {
    if (!gripper_closed_timer_active_) {
      gripper_closed_since_ = now;
      gripper_closed_timer_active_ = true;
    } else if ((now - gripper_closed_since_).seconds() >= gripper_grasp_hold_time_) {
      enableGraspConstraint();
      gripper_closed_timer_active_ = false;
    }
  } else {
    gripper_closed_timer_active_ = false;
  }

  if (grasp_constraint_active_ && both_open) {
    if (!gripper_open_timer_active_) {
      gripper_open_since_ = now;
      gripper_open_timer_active_ = true;
    } else if ((now - gripper_open_since_).seconds() >= gripper_grasp_hold_time_) {
      disableGraspConstraint();
      gripper_open_timer_active_ = false;
    }
  } else {
    gripper_open_timer_active_ = false;
  }
}

void AIWorkerBimanualMoveJController::enableGraspConstraint()
{
  captureCurrentGraspConstraint();
}

void AIWorkerBimanualMoveJController::disableGraspConstraint()
{
  grasp_constraint_active_ = false;
  gripper_closed_timer_active_ = false;
  gripper_open_timer_active_ = false;
  if (qp_filter_) {
    qp_filter_->setRigidGraspPoseConstraint(false, Eigen::Affine3d::Identity());
  }
}

cyclo_motion_controller::common::Vector6d AIWorkerBimanualMoveJController::computeDesiredTaskVelocity(
  const Eigen::Affine3d & current_pose, const Eigen::Affine3d & goal_pose) const
{
  cyclo_motion_controller::common::Vector6d desired_vel =
    cyclo_motion_controller::common::Vector6d::Zero();
  const Eigen::Vector3d position_error = goal_pose.translation() - current_pose.translation();
  const Eigen::Matrix3d rotation_error = goal_pose.linear() * current_pose.linear().transpose();
  const Eigen::AngleAxisd angle_axis_error(rotation_error);
  const Eigen::Vector3d orientation_error = angle_axis_error.axis() * angle_axis_error.angle();
  desired_vel.head<3>() = kp_grasp_position_ * position_error;
  desired_vel.tail<3>() = kp_grasp_orientation_ * orientation_error;
  return desired_vel;
}

Eigen::Affine3d AIWorkerBimanualMoveJController::blendPoses(
  const Eigen::Affine3d & pose_a, const Eigen::Affine3d & pose_b, const double blend) const
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

void AIWorkerBimanualMoveJController::captureCurrentGraspConstraint()
{
  if (!joint_state_received_) {
    return;
  }
  const Eigen::VectorXd q_feedback = commanded_state_initialized_ ? q_commanded_ : q_;
  kinematics_solver_->updateState(q_feedback, qdot_);
  const Eigen::Affine3d right_pose = kinematics_solver_->getPose(r_gripper_name_);
  const Eigen::Affine3d left_pose = kinematics_solver_->getPose(l_gripper_name_);
  grasp_right_to_left_ = right_pose.inverse() * left_pose;
  grasp_constraint_active_ = true;
}

void AIWorkerBimanualMoveJController::applyBimanualGoalProjection(
  Eigen::Affine3d & right_goal_pose,
  Eigen::Affine3d & left_goal_pose) const
{
  if (!grasp_constraint_active_) {
    return;
  }
  const Eigen::Affine3d object_from_right = right_goal_pose;
  const Eigen::Affine3d object_from_left = left_goal_pose * grasp_right_to_left_.inverse();
  const Eigen::Affine3d blended_object_pose =
    blendPoses(object_from_right, object_from_left, grasp_blend_ratio_);
  right_goal_pose = blended_object_pose;
  left_goal_pose = blended_object_pose * grasp_right_to_left_;
}

void AIWorkerBimanualMoveJController::assignArmSegment(
  const Eigen::VectorXd & source,
  const std::vector<std::string> & arm_joint_names,
  Eigen::VectorXd & destination) const
{
  for (const auto & joint_name : arm_joint_names) {
    const auto it = model_joint_index_map_.find(joint_name);
    if (it == model_joint_index_map_.end()) {
      continue;
    }
    destination[it->second] = source[it->second];
  }
}

void AIWorkerBimanualMoveJController::controlLoopCallback()
{
  if (!joint_state_received_ || !commanded_state_initialized_) {
    return;
  }
  if (jointStateTimedOut()) {
    joint_state_timeout_active_ = true;
    return;
  }

  try {
    const Eigen::VectorXd q_feedback = q_commanded_;
    kinematics_solver_->updateState(q_feedback, qdot_);

    Eigen::VectorXd q_ref = q_feedback;
    if (right_movej_target_initialized_) {
      assignArmSegment(right_movej_goal_, right_arm_joints_, q_ref);
    }
    if (left_movej_target_initialized_) {
      assignArmSegment(left_movej_goal_, left_arm_joints_, q_ref);
    }

    const Eigen::VectorXd desired_joint_vel = kp_joint_ * (q_ref - q_feedback);
    const Eigen::VectorXd joint_weight =
      Eigen::VectorXd::Ones(kinematics_solver_->getDof()) * weight_tracking_;
    const Eigen::VectorXd damping =
      Eigen::VectorXd::Ones(kinematics_solver_->getDof()) * weight_damping_;

    if (grasp_constraint_active_) {
      qp_filter_->setRigidGraspPoseConstraint(
        true,
        grasp_right_to_left_);
    } else {
      qp_filter_->setRigidGraspPoseConstraint(false, Eigen::Affine3d::Identity());
    }

    qp_filter_->setDesiredJointVel(desired_joint_vel);
    qp_filter_->setWeight(joint_weight, damping);

    Eigen::VectorXd optimal_velocities;
    if (!qp_filter_->getOptJointVel(optimal_velocities)) {
      return;
    }

    q_commanded_ = q_feedback + optimal_velocities * time_step_;
    publishTrajectory(q_commanded_);
    qdot_ = optimal_velocities;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Bimanual moveJ loop error: %s", e.what());
  }
}

bool AIWorkerBimanualMoveJController::jointStateTimedOut() const
{
  return joint_state_received_ &&
         (this->now() - last_joint_state_time_).seconds() > joint_state_timeout_;
}

void AIWorkerBimanualMoveJController::syncCommandStateToFeedback()
{
  q_commanded_ = q_;
  right_movej_goal_ = q_;
  left_movej_goal_ = q_;
}

void AIWorkerBimanualMoveJController::syncRightArmToFeedback()
{
  assignArmSegment(q_, right_arm_joints_, q_commanded_);
}

void AIWorkerBimanualMoveJController::syncLeftArmToFeedback()
{
  assignArmSegment(q_, left_arm_joints_, q_commanded_);
}

void AIWorkerBimanualMoveJController::publishTrajectory(const Eigen::VectorXd & q_command) const
{
  std::vector<int> left_arm_indices;
  std::vector<int> right_arm_indices;

  for (const auto & joint_name : left_arm_joints_) {
    const auto it = model_joint_index_map_.find(joint_name);
    if (it != model_joint_index_map_.end()) {
      left_arm_indices.push_back(it->second);
    }
  }
  for (const auto & joint_name : right_arm_joints_) {
    const auto it = model_joint_index_map_.find(joint_name);
    if (it != model_joint_index_map_.end()) {
      right_arm_indices.push_back(it->second);
    }
  }

  if (!left_arm_indices.empty()) {
    arm_l_pub_->publish(createTrajectoryMsgWithGripper(
      left_arm_joints_, q_command, left_arm_indices, left_gripper_joint_name_, left_gripper_position_));
  }
  if (!right_arm_indices.empty()) {
    arm_r_pub_->publish(createTrajectoryMsgWithGripper(
      right_arm_joints_, q_command, right_arm_indices, right_gripper_joint_name_, right_gripper_position_));
  }
}

trajectory_msgs::msg::JointTrajectory AIWorkerBimanualMoveJController::createTrajectoryMsgWithGripper(
  const std::vector<std::string> & arm_joint_names,
  const Eigen::VectorXd & positions,
  const std::vector<int> & arm_indices,
  const std::string & gripper_joint_name,
  double gripper_position) const
{
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.header.frame_id = "";
  traj_msg.joint_names = arm_joint_names;
  traj_msg.joint_names.push_back(gripper_joint_name);

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(trajectory_time_);
  for (int idx : arm_indices) {
    if (idx >= 0 && idx < static_cast<int>(positions.size())) {
      point.positions.push_back(positions[idx]);
      point.velocities.push_back(0.0);
    }
  }
  point.positions.push_back(gripper_position);
  point.velocities.push_back(0.0);
  traj_msg.points.push_back(point);
  return traj_msg;
}

}  // namespace cyclo_motion_controller_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cyclo_motion_controller_ros::AIWorkerBimanualMoveJController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
