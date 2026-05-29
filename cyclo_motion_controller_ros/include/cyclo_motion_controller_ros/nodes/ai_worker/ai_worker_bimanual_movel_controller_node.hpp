#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "common/type_define.hpp"
#include "controllers/ai_worker/ai_worker_bimanual_movel_controller.hpp"
#include "kinematics/kinematics_solver.hpp"

namespace cyclo_motion_controller_ros
{
class AIWorkerBimanualMoveLControllerNode : public rclcpp::Node
{
public:
  AIWorkerBimanualMoveLControllerNode();
  ~AIWorkerBimanualMoveLControllerNode();

private:
  void initializeJointConfig();
  void extractJointStates(const sensor_msgs::msg::JointState::SharedPtr & msg);
  void publishTrajectory(const Eigen::VectorXd & q_desired) const;
  void publishGripperPose(const Eigen::Affine3d & right_pose, const Eigen::Affine3d & left_pose);
  bool jointStateTimedOut() const;
  void syncCommandStateToFeedback();
  void syncArmStateToFeedback(
    const std::vector<std::string> & arm_joint_names,
    Eigen::VectorXd & destination) const;

  void rightGoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void leftGoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void virtualObjectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void graspCaptureCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void controlLoopCallback();

  Eigen::Affine3d poseMsgToEigen(const geometry_msgs::msg::PoseStamped & pose_msg) const;
  cyclo_motion_controller::common::Vector6d computeDesiredVelocity(
    const Eigen::Affine3d & current_pose,
    const Eigen::Affine3d & goal_pose) const;
  Eigen::Affine3d blendPoses(
    const Eigen::Affine3d & pose_a, const Eigen::Affine3d & pose_b, double blend) const;
  void captureCurrentGraspConstraint();
  void applyBimanualGoalProjection(
    Eigen::Affine3d & right_goal_pose,
    Eigen::Affine3d & left_goal_pose) const;

  trajectory_msgs::msg::JointTrajectory createArmTrajectoryMsg(
    const std::vector<std::string> & arm_joint_names,
    const Eigen::VectorXd & positions,
    const std::vector<int> & arm_indices) const;
  trajectory_msgs::msg::JointTrajectory createLiftTrajectoryMsg(
    const std::string & lift_joint_name, double position) const;

  double control_frequency_;
  double time_step_;
  double trajectory_time_;
  double kp_position_;
  double kp_orientation_;
  double weight_position_;
  double weight_orientation_;
  double weight_damping_;
  double slack_penalty_;
  double cbf_alpha_;
  double collision_buffer_;
  double collision_safe_distance_;
  double rigid_grasp_position_recovery_gain_;
  double rigid_grasp_orientation_recovery_gain_;
  double joint_state_timeout_;
  double goal_command_timeout_;
  double passive_hold_weight_scale_;
  double grasp_blend_ratio_;
  std::string joint_states_topic_;
  std::string right_goal_pose_topic_;
  std::string left_goal_pose_topic_;
  std::string virtual_object_pose_topic_;
  std::string grasp_capture_topic_;
  std::string right_traj_topic_;
  std::string left_traj_topic_;
  std::string lift_topic_;
  double lift_vel_bound_;
  std::string r_gripper_pose_topic_;
  std::string l_gripper_pose_topic_;
  std::string r_gripper_name_;
  std::string l_gripper_name_;
  std::string urdf_path_;
  std::string srdf_path_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr virtual_object_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp_capture_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_r_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_l_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr lift_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr r_gripper_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr l_gripper_pose_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  std::shared_ptr<cyclo_motion_controller::kinematics::KinematicsSolver> kinematics_solver_;
  std::shared_ptr<cyclo_motion_controller::controllers::AIWorkerBimanualMoveLController>
  qp_controller_;

  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;
  Eigen::VectorXd q_desired_;

  Eigen::Affine3d right_gripper_pose_;
  Eigen::Affine3d left_gripper_pose_;
  Eigen::Affine3d right_goal_pose_;
  Eigen::Affine3d left_goal_pose_;
  Eigen::Affine3d virtual_object_goal_pose_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d grasp_object_to_right_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d grasp_object_to_left_ = Eigen::Affine3d::Identity();

  bool right_goal_pose_received_ = false;
  bool left_goal_pose_received_ = false;
  bool right_goal_pose_updated_ = false;
  bool left_goal_pose_updated_ = false;
  bool virtual_object_goal_received_ = false;
  bool virtual_object_goal_updated_ = false;
  bool joint_state_received_ = false;
  bool q_desired_initialized_ = false;
  bool joint_state_timeout_active_ = false;
  bool grasp_constraint_active_ = false;

  rclcpp::Time last_joint_state_time_;
  rclcpp::Time last_right_goal_cmd_time_;
  rclcpp::Time last_left_goal_cmd_time_;

  std::vector<std::string> left_arm_joints_;
  std::vector<std::string> right_arm_joints_;
  std::string lift_joint_;
  int lift_joint_index_ = -1;
  std::unordered_map<std::string, int> joint_index_map_;
  std::vector<std::string> model_joint_names_;
  std::unordered_map<std::string, int> model_joint_index_map_;
};
}  // namespace cyclo_motion_controller_ros
