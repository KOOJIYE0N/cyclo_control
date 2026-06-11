#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robotis_interfaces/msg/move_l.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace
{
using namespace std::chrono_literals;

geometry_msgs::msg::PoseStamped makePoseMsg(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const Eigen::Affine3d & pose)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.pose.position.x = pose.translation().x();
  msg.pose.position.y = pose.translation().y();
  msg.pose.position.z = pose.translation().z();

  const Eigen::Quaterniond quat(pose.linear());
  msg.pose.orientation.x = quat.x();
  msg.pose.orientation.y = quat.y();
  msg.pose.orientation.z = quat.z();
  msg.pose.orientation.w = quat.w();
  return msg;
}

robotis_interfaces::msg::MoveL makeMoveLMsg(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const Eigen::Affine3d & pose,
  const double duration)
{
  robotis_interfaces::msg::MoveL msg;
  msg.pose = makePoseMsg(frame_id, stamp, pose);
  msg.time_from_start = rclcpp::Duration::from_seconds(duration);
  return msg;
}

Eigen::Affine3d poseMsgToEigen(const geometry_msgs::msg::PoseStamped & msg)
{
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translation() << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  const Eigen::Quaterniond quat(
    msg.pose.orientation.w,
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z);
  pose.linear() = quat.normalized().toRotationMatrix();
  return pose;
}

Eigen::Affine3d blendPoses(
  const Eigen::Affine3d & start,
  const Eigen::Affine3d & goal,
  const double alpha)
{
  const double t = std::clamp(alpha, 0.0, 1.0);
  Eigen::Affine3d blended = Eigen::Affine3d::Identity();
  blended.translation() = (1.0 - t) * start.translation() + t * goal.translation();

  const Eigen::Quaterniond start_quat(start.linear());
  const Eigen::Quaterniond goal_quat(goal.linear());
  blended.linear() = start_quat.slerp(t, goal_quat).normalized().toRotationMatrix();
  return blended;
}

Eigen::Affine3d makePose(
  const double x,
  const double y,
  const double z,
  const double qx,
  const double qy,
  const double qz,
  const double qw)
{
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translation() << x, y, z;
  pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
  return pose;
}
}  // namespace

namespace cyclo_motion_controller_ros
{
class BimanualPanelRollDemoNode : public rclcpp::Node
{
public:
  BimanualPanelRollDemoNode()
  : Node("bimanual_panel_roll_demo_node")
  {
    frame_id_ = this->declare_parameter("frame_id", std::string("base_link"));
    publish_frequency_ = this->declare_parameter("publish_frequency", 50.0);
    move_duration_ = this->declare_parameter("move_duration", 5.0);
    hold_duration_ = this->declare_parameter("hold_duration", 5.0);
    grasp_settle_duration_ = this->declare_parameter("grasp_settle_duration", 1.0);
    roll_amplitude_deg_ = this->declare_parameter("roll_amplitude_deg", 20.0);
    roll_period_ = this->declare_parameter("roll_period", 8.0);
    close_gripper_position_ = this->declare_parameter("close_gripper_position", 1.2);
    gripper_trajectory_time_ = this->declare_parameter("gripper_trajectory_time", 0.0);

    right_movel_topic_ =
      this->declare_parameter("right_movel_topic", std::string("/r_goal_move"));
    left_movel_topic_ =
      this->declare_parameter("left_movel_topic", std::string("/l_goal_move"));
    right_gripper_pose_topic_ =
      this->declare_parameter("right_gripper_pose_topic", std::string("/r_gripper_pose"));
    left_gripper_pose_topic_ =
      this->declare_parameter("left_gripper_pose_topic", std::string("/l_gripper_pose"));
    virtual_object_movel_topic_ = this->declare_parameter(
      "virtual_object_movel_topic", std::string("/virtual_object_goal_move"));
    grasp_capture_topic_ =
      this->declare_parameter("grasp_capture_topic", std::string("/capture_grasp"));
    right_gripper_command_topic_ = this->declare_parameter(
      "right_gripper_command_topic",
      std::string("/leader/joint_trajectory_command_broadcaster_right/joint_trajectory"));
    left_gripper_command_topic_ = this->declare_parameter(
      "left_gripper_command_topic",
      std::string("/leader/joint_trajectory_command_broadcaster_left/joint_trajectory"));
    right_gripper_joint_ =
      this->declare_parameter("right_gripper_joint", std::string("gripper_r_joint1"));
    left_gripper_joint_ =
      this->declare_parameter("left_gripper_joint", std::string("gripper_l_joint1"));

    right_target_pose_ = makePose(0.3, -0.1, 1.2, 0.5, -0.5, 0.5, 0.5);
    left_target_pose_ = makePose(0.3, 0.1, 1.2, -0.5, -0.5, -0.5, 0.5);

    right_goal_pub_ = this->create_publisher<robotis_interfaces::msg::MoveL>(
      right_movel_topic_, 10);
    left_goal_pub_ = this->create_publisher<robotis_interfaces::msg::MoveL>(
      left_movel_topic_, 10);
    virtual_object_pub_ = this->create_publisher<robotis_interfaces::msg::MoveL>(
      virtual_object_movel_topic_, 10);
    grasp_capture_pub_ = this->create_publisher<std_msgs::msg::Bool>(grasp_capture_topic_, 10);
    right_gripper_command_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      right_gripper_command_topic_, 10);
    left_gripper_command_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      left_gripper_command_topic_, 10);

    right_gripper_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      right_gripper_pose_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        right_current_pose_ = poseMsgToEigen(*msg);
        right_pose_received_ = true;
      });
    left_gripper_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      left_gripper_pose_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        left_current_pose_ = poseMsgToEigen(*msg);
        left_pose_received_ = true;
      });

    const int timer_period_ms =
      std::max(1, static_cast<int>(std::round(1000.0 / std::max(1.0, publish_frequency_))));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_period_ms),
      std::bind(&BimanualPanelRollDemoNode::timerCallback, this));
  }

private:
  enum class Phase
  {
    WAIT_FOR_FEEDBACK,
    MOVE_TO_PANEL,
    HOLD_PANEL,
    CLOSE_GRIPPERS,
    CAPTURE_GRASP,
    OSCILLATE_PANEL
  };

  void timerCallback()
  {
    const rclcpp::Time now = this->now();
    if (phase_ == Phase::WAIT_FOR_FEEDBACK) {
      if (!right_pose_received_ || !left_pose_received_) {
        return;
      }
      right_move_start_pose_ = right_current_pose_;
      left_move_start_pose_ = left_current_pose_;
      publishHandGoals(right_target_pose_, left_target_pose_, now, move_duration_);
      phase_start_time_ = now;
      phase_ = Phase::MOVE_TO_PANEL;
      RCLCPP_INFO(this->get_logger(), "Moving to panel grasp pose.");
    }

    if (phase_ == Phase::MOVE_TO_PANEL) {
      if (elapsed(now) >= move_duration_) {
        phase_start_time_ = now;
        phase_ = Phase::HOLD_PANEL;
        RCLCPP_INFO(this->get_logger(), "Holding panel pose before closing grippers.");
      }
      return;
    }

    if (phase_ == Phase::HOLD_PANEL) {
      publishHandGoals(right_target_pose_, left_target_pose_, now, 0.0);
      if (elapsed(now) >= hold_duration_) {
        publishGripperCommands();
        phase_start_time_ = now;
        phase_ = Phase::CLOSE_GRIPPERS;
        RCLCPP_INFO(this->get_logger(), "Closing grippers.");
      }
      return;
    }

    if (phase_ == Phase::CLOSE_GRIPPERS) {
      publishHandGoals(right_target_pose_, left_target_pose_, now, 0.0);
      publishGripperCommands();
      if (elapsed(now) >= gripper_trajectory_time_) {
        std_msgs::msg::Bool grasp_msg;
        grasp_msg.data = true;
        grasp_capture_pub_->publish(grasp_msg);
        phase_start_time_ = now;
        phase_ = Phase::CAPTURE_GRASP;
        RCLCPP_INFO(this->get_logger(), "Capturing rigid grasp constraint.");
      }
      return;
    }

    if (phase_ == Phase::CAPTURE_GRASP) {
      publishVirtualObjectGoal(currentVirtualObjectPose(), now, 0.0);
      if (elapsed(now) >= grasp_settle_duration_) {
        object_roll_anchor_pose_ = currentVirtualObjectPose();
        phase_start_time_ = now;
        phase_ = Phase::OSCILLATE_PANEL;
        RCLCPP_INFO(this->get_logger(), "Oscillating virtual panel roll.");
      }
      return;
    }

    publishVirtualObjectGoal(makeRolledObjectGoal(now), now, 0.0);
  }

  double elapsed(const rclcpp::Time & now) const
  {
    return (now - phase_start_time_).seconds();
  }

  void publishHandGoals(
    const Eigen::Affine3d & right_pose,
    const Eigen::Affine3d & left_pose,
    const rclcpp::Time & stamp,
    const double duration) const
  {
    right_goal_pub_->publish(makeMoveLMsg(frame_id_, stamp, right_pose, duration));
    left_goal_pub_->publish(makeMoveLMsg(frame_id_, stamp, left_pose, duration));
  }

  void publishVirtualObjectGoal(
    const Eigen::Affine3d & pose,
    const rclcpp::Time & stamp,
    const double duration) const
  {
    virtual_object_pub_->publish(makeMoveLMsg(frame_id_, stamp, pose, duration));
  }

  void publishGripperCommand(
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & publisher,
    const std::string & joint_name) const
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = {joint_name};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {close_gripper_position_};
    point.velocities = {0.0};
    point.time_from_start = rclcpp::Duration::from_seconds(gripper_trajectory_time_);
    trajectory.points.push_back(point);
    publisher->publish(trajectory);
  }

  void publishGripperCommands() const
  {
    publishGripperCommand(right_gripper_command_pub_, right_gripper_joint_);
    publishGripperCommand(left_gripper_command_pub_, left_gripper_joint_);
  }

  Eigen::Affine3d currentVirtualObjectPose() const
  {
    return blendPoses(right_current_pose_, left_current_pose_, 0.5);
  }

  Eigen::Affine3d makeRolledObjectGoal(const rclcpp::Time & now) const
  {
    const double amplitude_rad = roll_amplitude_deg_ * M_PI / 180.0;
    const double period = std::max(1e-6, roll_period_);
    const double roll = amplitude_rad * std::sin(2.0 * M_PI * elapsed(now) / period);
    const Eigen::AngleAxisd roll_rotation(roll, Eigen::Vector3d::UnitX());

    Eigen::Affine3d goal = object_roll_anchor_pose_;
    goal.linear() = roll_rotation.toRotationMatrix() * object_roll_anchor_pose_.linear();
    return goal;
  }

  std::string frame_id_;
  double publish_frequency_;
  double move_duration_;
  double hold_duration_;
  double grasp_settle_duration_;
  double roll_amplitude_deg_;
  double roll_period_;
  double close_gripper_position_;
  double gripper_trajectory_time_;

  std::string right_movel_topic_;
  std::string left_movel_topic_;
  std::string right_gripper_pose_topic_;
  std::string left_gripper_pose_topic_;
  std::string virtual_object_movel_topic_;
  std::string grasp_capture_topic_;
  std::string right_gripper_command_topic_;
  std::string left_gripper_command_topic_;
  std::string right_gripper_joint_;
  std::string left_gripper_joint_;

  Eigen::Affine3d right_current_pose_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d left_current_pose_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d right_move_start_pose_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d left_move_start_pose_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d right_target_pose_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d left_target_pose_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d object_roll_anchor_pose_ = Eigen::Affine3d::Identity();

  bool right_pose_received_ = false;
  bool left_pose_received_ = false;
  Phase phase_ = Phase::WAIT_FOR_FEEDBACK;
  rclcpp::Time phase_start_time_;

  rclcpp::Publisher<robotis_interfaces::msg::MoveL>::SharedPtr right_goal_pub_;
  rclcpp::Publisher<robotis_interfaces::msg::MoveL>::SharedPtr left_goal_pub_;
  rclcpp::Publisher<robotis_interfaces::msg::MoveL>::SharedPtr virtual_object_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasp_capture_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_gripper_command_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_gripper_command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_gripper_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_gripper_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace cyclo_motion_controller_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cyclo_motion_controller_ros::BimanualPanelRollDemoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
