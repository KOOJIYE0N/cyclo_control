#pragma once

#include <memory>
#include <string>
#include <map>

#include <Eigen/Geometry>

#include "common/type_define.hpp"
#include "kinematics/kinematics_solver.hpp"
#include "optimization/qp_base.hpp"

namespace cyclo_motion_controller
{
namespace controllers
{
class AIWorkerBimanualController : public cyclo_motion_controller::optimization::QPBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AIWorkerBimanualController(
    std::shared_ptr<cyclo_motion_controller::kinematics::KinematicsSolver> robot_data,
    const double dt);

  void setWeight(
    const std::map<std::string, cyclo_motion_controller::common::Vector6d> & link_w_tracking,
    const Eigen::VectorXd & w_damping);
  void setDesiredTaskVel(
    const std::map<std::string, cyclo_motion_controller::common::Vector6d> & link_xdot_desired);
  void setControllerParams(
    const double slack_penalty, const double cbf_alpha,
    const double buffer_distance, const double safe_distance);
  void setConstraintLinks(const std::string & right_link, const std::string & left_link);
  void setRigidGraspConstraint(const bool active, const Eigen::Vector3d & right_to_left_world);
  void setRigidGraspConstraint(
    const bool active,
    const Eigen::Affine3d & right_to_left_in_right,
    const double position_recovery_gain,
    const double orientation_recovery_gain);
  bool getOptJointVel(Eigen::VectorXd & opt_qdot);

private:
  struct QPIndex
  {
    int qdot_start;
    int slack_q_min_start;
    int slack_q_max_start;
    int slack_sing_start;
    int slack_sel_col_start;

    int qdot_size;
    int slack_q_min_size;
    int slack_q_max_size;
    int slack_sing_size;
    int slack_sel_col_size;

    int con_q_min_start;
    int con_q_max_start;
    int con_sing_start;
    int con_sel_col_start;

    int con_q_min_size;
    int con_q_max_size;
    int con_sing_size;
    int con_sel_col_size;

    int eq_grasp_start;
    int eq_grasp_size;
  } si_index_;

  std::shared_ptr<cyclo_motion_controller::kinematics::KinematicsSolver> robot_data_;
  double dt_;
  int joint_dof_;

  std::map<std::string, cyclo_motion_controller::common::Vector6d> link_xdot_desired_;
  std::map<std::string, cyclo_motion_controller::common::Vector6d> link_w_tracking_;
  Eigen::VectorXd w_damping_;
  double slack_penalty_;
  double cbf_alpha_;
  double collision_buffer_;
  double collision_safe_distance_;
  bool rigid_grasp_active_ = false;
  Eigen::Vector3d rigid_right_to_left_world_ = Eigen::Vector3d::Zero();
  bool rigid_grasp_use_relative_transform_ = false;
  Eigen::Affine3d rigid_right_to_left_in_right_ = Eigen::Affine3d::Identity();
  double rigid_grasp_position_recovery_gain_ = 0.0;
  double rigid_grasp_orientation_recovery_gain_ = 0.0;
  std::string right_constraint_link_ = "arm_r_link7";
  std::string left_constraint_link_ = "arm_l_link7";

  void setCost() override;
  void setBoundConstraint() override;
  void setIneqConstraint() override;
  void setEqConstraint() override;
};
}  // namespace controllers
}  // namespace cyclo_motion_controller
