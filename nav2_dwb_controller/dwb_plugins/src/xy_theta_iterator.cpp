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

#include "dwb_plugins/xy_theta_iterator.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "nav_2d_utils/parameters.hpp"
#include "nav2_util/node_utils.hpp"

#define EPSILON 1E-5

namespace dwb_plugins
{
void XYThetaIterator::initialize(
  const nav2_util::LifecycleNode::SharedPtr & nh,
  KinematicsHandler::Ptr kinematics,
  const std::string & plugin_name)
{
  kinematics_handler_ = kinematics;

  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".vx_samples", rclcpp::ParameterValue(20));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".vy_samples", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".vtheta_samples", rclcpp::ParameterValue(20));

  nh->get_parameter(plugin_name + ".vx_samples", vx_samples_);
  nh->get_parameter(plugin_name + ".vy_samples", vy_samples_);
  nh->get_parameter(plugin_name + ".vtheta_samples", vtheta_samples_);
}


/*--
현재 로봇의 cmd_vel과 sim_time을 아규먼트로 받는다.
sim_time은 얼마나 먼 시간까지 내다볼지에 대한 아규먼트이다.
로봇의 가속도 & 각가속도와 내다볼 시간의 조합을 통해 다음 제어 주기에 도달 가능한 v_x, v_y, w_z 리스트들을 생성할 셋팅을 한다.
*/
void XYThetaIterator::startNewIteration(
  const nav_2d_msgs::msg::Twist2D & current_velocity,
  double dt)
{
  KinematicParameters kinematics = kinematics_handler_->getKinematics();


/*--
로봇의 현재 cmd_vel, max & min cmd_vel, acc & decel, sim_time, sample 개수를 아규먼트로 넣어주고,
현재 로봇 cmd_vel 로부터 다음 제어 주기에 도달 가능한 v_x, v_y, w_z 리스트들을 생성할 셋팅을 한다.
*/
  x_it_ = std::make_shared<OneDVelocityIterator>(
    current_velocity.x,
    kinematics.getMinX(), kinematics.getMaxX(),
    kinematics.getAccX(), kinematics.getDecelX(),
    dt, vx_samples_);

  y_it_ = std::make_shared<OneDVelocityIterator>(
    current_velocity.y,
    kinematics.getMinY(), kinematics.getMaxY(),
    kinematics.getAccY(), kinematics.getDecelY(),
    dt, vy_samples_);

  th_it_ = std::make_shared<OneDVelocityIterator>(
    current_velocity.theta,
    kinematics.getMinTheta(), kinematics.getMaxTheta(),
    kinematics.getAccTheta(), kinematics.getDecelTheta(),
    dt, vtheta_samples_);


  if (!isValidVelocity())
  {
    iterateToValidVelocity();
  }
}


bool XYThetaIterator::isValidSpeed(double x, double y, double theta)
{
  KinematicParameters kinematics = kinematics_handler_->getKinematics();
  double vmag_sq = x * x + y * y;
  if (kinematics.getMaxSpeedXY() >= 0.0 && vmag_sq > kinematics.getMaxSpeedXY_SQ() + EPSILON) {
    return false;
  }
  if (kinematics.getMinSpeedXY() >= 0.0 && vmag_sq + EPSILON < kinematics.getMinSpeedXY_SQ() &&
    kinematics.getMinSpeedTheta() >= 0.0 && fabs(theta) + EPSILON < kinematics.getMinSpeedTheta())
  {
    return false;
  }
  if (vmag_sq == 0.0 && th_it_->getVelocity() == 0.0) {
    return false;
  }
  return true;
}

bool XYThetaIterator::isValidVelocity()
{
  return isValidSpeed(
    x_it_->getVelocity(), y_it_->getVelocity(),
    th_it_->getVelocity());
}


/*--
v_x, v_y, w_z 리스트의 속도 지원자들 중,
아직 체크하지 않은 원소들이 있는지 확인한다.
(v_x, v_y, w_z)의 한 속도 세트를 만들 때,
w_z -> v_y -> v_x 순으로 지정되기 때문에
v_x 리스트만 아직 체크하지 않은 원소가 있는지 최종적으로 확인하면 된다.
*/
bool XYThetaIterator::hasMoreTwists()
{
  return x_it_ && !x_it_->isFinished();
}


/*--
(v_x, v_y, w_z) 한 속도 벡터 세트를 반환한다.
즉, 다음 속도 지원자를 반환한다.
*/
nav_2d_msgs::msg::Twist2D XYThetaIterator::nextTwist()
{
  nav_2d_msgs::msg::Twist2D velocity;


/*--
(v_x, v_y, w_z) 각 1차원 속도 리스트의 현재 인덱스 원소를 받환받는다.
*/
  velocity.x = x_it_->getVelocity();
  velocity.y = y_it_->getVelocity();
  velocity.theta = th_it_->getVelocity();


/*--
(v_x, v_y, w_z) 각 1차원 속도 리스트의 인덱스를 업데이트한다.
업데이트 하는 순서는,
1. w_z의 min_vel ~ max_vel
2. v_y의 min_vel ~ max_vel
3. v_x의 min_vel ~ max_vel
순서이다.
*/
  iterateToValidVelocity();

  return velocity;
}


/*--
(v_x, v_y, w_z) 각 1차원 속도 리스트의 인덱스를 업데이트한다.
업데이트 하는 순서는,
1. w_z의 min_vel ~ max_vel
2. v_y의 min_vel ~ max_vel
3. v_x의 min_vel ~ max_vel
순서이다.
*/
void XYThetaIterator::iterateToValidVelocity()
{
  bool valid = false;

  while (!valid && hasMoreTwists())
  {
    ++(*th_it_);

    if (th_it_->isFinished())
    {
      th_it_->reset();
      ++(*y_it_);

      if (y_it_->isFinished())
      {
        y_it_->reset();
        ++(*x_it_);
      }
    }

    valid = isValidVelocity();
  }
}

}  // namespace dwb_plugins
