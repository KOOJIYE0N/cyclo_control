// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// This file is derived from `dyros_robot_controller` project:
// https://github.com/JunHeonYoon/dyros_robot_controller
//
// Original work Copyright (c) 2025 JunHeonYoon, licensed under the
// Apache License 2.0. Modifications in this file are Copyright 2026
// ROBOTIS CO., LTD.
//
// Author: Yeonguk Kim

#pragma once

#include <Eigen/Geometry>

#include <memory>
#include <string>

#include "kinematics/kinematics_solver.hpp"
#include "optimization/qp_base.hpp"

namespace cyclo_motion_controller
{
namespace controllers
{
class AIWorkerBimanualMoveJController : public cyclo_motion_controller::optimization::QPBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AIWorkerBimanualMoveJController(
    std::shared_ptr<cyclo_motion_controller::kinematics::KinematicsSolver> robot_data,
    double dt);

  void setWeight(const Eigen::VectorXd & w_tracking, const Eigen::VectorXd & w_damping);
  void setDesiredJointVel(const Eigen::VectorXd & qdot_desired);
  void setControllerParams(
    double slack_penalty, double cbf_alpha, double buffer_distance, double safe_distance);
  void setConstraintLinks(const std::string & right_link, const std::string & left_link);
  void setRigidGraspPoseConstraint(
    bool active, const Eigen::Affine3d & right_to_left_in_right);
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

  Eigen::VectorXd qdot_desired_;
  Eigen::VectorXd w_joint_tracking_;
  Eigen::VectorXd w_damping_;
  double slack_penalty_;
  double cbf_alpha_;
  double collision_buffer_;
  double collision_safe_distance_;
  bool rigid_grasp_active_ = false;
  Eigen::Affine3d rigid_right_to_left_in_right_ = Eigen::Affine3d::Identity();
  std::string right_constraint_link_ = "arm_r_link7";
  std::string left_constraint_link_ = "arm_l_link7";

  Eigen::VectorXd projectDesiredVelocityToGraspManifold(
    const Eigen::VectorXd & qdot_desired) const;
  void setCost() override;
  void setBoundConstraint() override;
  void setIneqConstraint() override;
  void setEqConstraint() override;
};
}  // namespace controllers
}  // namespace cyclo_motion_controller
