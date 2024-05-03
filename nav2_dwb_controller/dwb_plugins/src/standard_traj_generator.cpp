/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "dwb_plugins/standard_traj_generator.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <memory>
#include "dwb_plugins/xy_theta_iterator.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"

namespace dwb_plugins
{

void StandardTrajectoryGenerator::initialize(
  const nav2_util::LifecycleNode::SharedPtr & nh,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  kinematics_handler_ = std::make_shared<KinematicsHandler>();
  kinematics_handler_->initialize(nh, plugin_name_);
  initializeIterator(nh);

  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".sim_time", rclcpp::ParameterValue(1.7));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".discretize_by_time", rclcpp::ParameterValue(false));

  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".time_granularity", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".linear_granularity", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".angular_granularity", rclcpp::ParameterValue(0.025));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".include_last_point", rclcpp::ParameterValue(true));

  /*
   * If discretize_by_time, then sim_granularity represents the amount of time that should be between
   *  two successive points on the trajectory.
   *
   * If discretize_by_time is false, then sim_granularity is the maximum amount of distance between
   *  two successive points on the trajectory, and angular_sim_granularity is the maximum amount of
   *  angular distance between two successive points.
   */
  nh->get_parameter(plugin_name + ".sim_time", sim_time_);
  nh->get_parameter(plugin_name + ".discretize_by_time", discretize_by_time_);
  nh->get_parameter(plugin_name + ".time_granularity", time_granularity_);
  nh->get_parameter(plugin_name + ".linear_granularity", linear_granularity_);
  nh->get_parameter(plugin_name + ".angular_granularity", angular_granularity_);
  nh->get_parameter(plugin_name + ".include_last_point", include_last_point_);
}

void StandardTrajectoryGenerator::initializeIterator(
  const nav2_util::LifecycleNode::SharedPtr & nh)
{
  velocity_iterator_ = std::make_shared<XYThetaIterator>();
  velocity_iterator_->initialize(nh, kinematics_handler_, plugin_name_);
}


/*--
현재 로봇의 cmd_vel을 아규먼트로 받고,
현재 cmd_vel로부터 다음 제어 주기에 도달 가능한 v_x, v_y, w_z의 리스트들을 생성할 셋팅을 한다.
(Dynamic Window)
*/
void StandardTrajectoryGenerator::startNewIteration(
  const nav_2d_msgs::msg::Twist2D & current_velocity)
{

/*--
현재 로봇의 cmd_vel과 sim_time을 아규먼트로 넣어준다.
sim_time은 얼마나 먼 시간까지 내다볼지에 대한 아규먼트이다.
로봇의 가속도 & 각가속도와 내다볼 시간의 조합을 통해 다음 제어 주기에 도달 가능한 v_x, v_y, w_z 리스트들을 생성할 셋팅을 한다.
*/
  velocity_iterator_->startNewIteration(current_velocity, sim_time_);
}


/*--
v_x, v_y, w_z 리스트의 속도 지원자들 중,
아직 체크하지 않은 원소들이 있는지 확인한다.
*/
bool StandardTrajectoryGenerator::hasMoreTwists()
{
  return velocity_iterator_->hasMoreTwists();
}


/*--
(v_x, v_y, w_z) 한 속도 벡터 세트를 반환한다.
즉, 다음 속도 지원자를 반환한다.
*/
nav_2d_msgs::msg::Twist2D StandardTrajectoryGenerator::nextTwist()
{
  return velocity_iterator_->nextTwist();
}


/*--
아규먼트로 다음 속도 지원자를 받으며,
생성할 궤적을 소분할 개수 만큼의 size를 가지는 리스트를 반환한다.
아규먼트로 받는 속도 지원자의 용도는,
1. 만약 discretize_by_time param이 true인 경우,
    궤적은 (sim_time / time_granularity) 으로 소분된다.
2. 만약 false인 경우,
    궤적은 ((속도 지원자 * sim_time) / linear or angular_granularity) 으로 소분된다.

리스트의 각 인덱스에는 궤적의 각 소분된 간격 간의 dt가 들어간다. (모두 동일한 값)
*/
std::vector<double> StandardTrajectoryGenerator::getTimeSteps(
  const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  std::vector<double> steps;


/*--
discretize_by_time param이 true인 경우,
궤적은 (sim_time / time_granularity) 으로 소분된다.
*/
  if (discretize_by_time_)
  {
    steps.resize(ceil(sim_time_ / time_granularity_));
  }


/*--
discretize_by_time param이 false인 경우,
궤적은 ((속도 지원자 * sim_time) / linear or angular_granularity) 으로 소분된다.
*/
  else
  {  // discretize by distance

    /*
    sqrt(x*x + y*y)
    */
    double vmag = hypot(cmd_vel.x, cmd_vel.y);


    // the distance the robot would travel in sim_time if it did not change velocity

/*--
속도 지원자 * sim_time
속도 지원자가 값이 클수록 궤적은 더욱 세밀하게 소분된다.
*/
    double projected_linear_distance = vmag * sim_time_;
    // the angle the robot would rotate in sim_time
    double projected_angular_distance = fabs(cmd_vel.theta) * sim_time_;


    // Pick the maximum of the two
    int num_steps = ceil(
      std::max(
        projected_linear_distance / linear_granularity_,
        projected_angular_distance / angular_granularity_));

    steps.resize(num_steps);
  }

  if (steps.size() == 0)
  {
    steps.resize(1);
  }


/*--
리스트의 각 인덱스에는 궤적의 각 소분된 간격 간의 dt가 들어간다. (모두 동일한 값)
궤적은 로봇이 sim_time 만큼 주행했을 때의 궤적이다.
*/
  std::fill(steps.begin(), steps.end(), sim_time_ / steps.size());

  return steps;
}


/*--
아규먼트로 로봇의 현재 pose, 현재 cmd_vel, 다음 속도 지원자를 받는다.
로봇이 해당 pose 에 있을 경우,
다음 속도 지원자를 sim_time 만큼 적용하였을 경우의 궤적을 반환한다.
*/
dwb_msgs::msg::Trajectory2D StandardTrajectoryGenerator::generateTrajectory(
  const geometry_msgs::msg::Pose2D & start_pose,
  const nav_2d_msgs::msg::Twist2D & start_vel,
  const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  dwb_msgs::msg::Trajectory2D traj;
  traj.velocity = cmd_vel;


  //  simulate the trajectory
  geometry_msgs::msg::Pose2D pose = start_pose;
  nav_2d_msgs::msg::Twist2D vel = start_vel;
  double running_time = 0.0;


/*--
아규먼트로 다음 속도 지원자를 넣어주고,
생성할 궤적을 소분할 개수 만큼의 size를 가지는 리스트를 반환받는다.
아규먼트로 넘겨주는 속도 지원자의 용도는,
1. 만약 discretize_by_time param이 true인 경우,
    궤적은 (sim_time / time_granularity) 으로 소분된다.
2. 만약 false인 경우,
    궤적은 ((속도 지원자 * sim_time) / linear or angular_granularity) 으로 소분된다.

반환되는 리스트의 각 인덱스에는 궤적의 각 소분된 간격 간의 dt가 들어간다. (모두 동일한 값)
*/
  std::vector<double> steps = getTimeSteps(cmd_vel);


/*--
궤적의 초기 pose를 push한다.
*/
  traj.poses.push_back(start_pose);


/*--
궤적의 소분된 개수만큼 반복문을 실행하며,
각 dt마다 로봇의 새로운 cmd_vel 및 pose가 업데이트 된다.
모든 반복문을 실행하였을 때의 pose를 연결하면,
로봇의 궤적이 된다.
*/
  for (double dt : steps)
  {

    //  calculate velocities

/*--
아규먼트로 다음 속도 지원자, 로봇의 현재 cmd_vel 및 dt를 넘겨준다.
로봇의 acc & decel을 이용하여 현재 cmd_vel을 dt 시간 동안 점진적으로 가속시킨다.
다음 속도 지원자까지만 가속되며, 따라서 다음 속도 지원자는 max or min cmd_vel 로써 사용된다.
*/
    vel = computeNewVelocity(cmd_vel, vel, dt);


    //  update the position of the robot using the velocities passed in

/*--
아규먼트로 로봇의 현재 pose, 업데이트 된 cmd_vel, dt를 넘겨준다.
로봇이 현재 pose에서 해당 cmd_vel로 dt만큼 이동하였을 때의 로봇의 새로운 pose를 반환받는다.
*/
    pose = computeNewPosition(pose, vel, dt);


/*--
업데이트 된 pose를 궤적에 push한다.
*/
    traj.poses.push_back(pose);


    traj.time_offsets.push_back(rclcpp::Duration::from_seconds(running_time));
    running_time += dt;
  }  //  end for simulation steps


  if (include_last_point_)
  {
    traj.poses.push_back(pose);
    traj.time_offsets.push_back(rclcpp::Duration::from_seconds(running_time));
  }


  return traj;
}

/**
 * change vel using acceleration limits to converge towards sample_target-vel
 */

/*--
아규먼트로 다음 속도 지원자, 로봇의 현재 cmd_vel 및 dt를 받는다.
로봇의 acc & decel을 이용하여 현재 cmd_vel을 dt 시간 동안 점진적으로 가속시킨다.
다음 속도 지원자까지만 가속되며, 따라서 다음 속도 지원자는 max or min cmd_vel 로써 사용된다.
*/
nav_2d_msgs::msg::Twist2D StandardTrajectoryGenerator::computeNewVelocity(
  const nav_2d_msgs::msg::Twist2D & cmd_vel,
  const nav_2d_msgs::msg::Twist2D & start_vel, const double dt)
{
  KinematicParameters kinematics = kinematics_handler_->getKinematics();
  nav_2d_msgs::msg::Twist2D new_vel;


  new_vel.x = projectVelocity(
    start_vel.x, kinematics.getAccX(),
    kinematics.getDecelX(), dt, cmd_vel.x);

  new_vel.y = projectVelocity(
    start_vel.y, kinematics.getAccY(),
    kinematics.getDecelY(), dt, cmd_vel.y);

  new_vel.theta = projectVelocity(
    start_vel.theta,
    kinematics.getAccTheta(), kinematics.getDecelTheta(),
    dt, cmd_vel.theta);


  return new_vel;
}


/*--
아규먼트로 로봇의 현재 pose, 업데이트 된 cmd_vel, dt를 받는다.
로봇이 현재 pose에서 해당 cmd_vel로 dt만큼 이동하였을 때의 로봇의 새로운 pose를 반환한다.
*/
geometry_msgs::msg::Pose2D StandardTrajectoryGenerator::computeNewPosition(
  const geometry_msgs::msg::Pose2D start_pose,
  const nav_2d_msgs::msg::Twist2D & vel, const double dt)
{
  geometry_msgs::msg::Pose2D new_pose;


  new_pose.x = start_pose.x +
    (vel.x * cos(start_pose.theta) + vel.y * cos(M_PI_2 + start_pose.theta)) * dt;

  new_pose.y = start_pose.y +
    (vel.x * sin(start_pose.theta) + vel.y * sin(M_PI_2 + start_pose.theta)) * dt;

  new_pose.theta = start_pose.theta + vel.theta * dt;


  return new_pose;
}

}  // namespace dwb_plugins

PLUGINLIB_EXPORT_CLASS(
  dwb_plugins::StandardTrajectoryGenerator,
  dwb_core::TrajectoryGenerator)
