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

#include "dwb_critics/rotate_to_goal.hpp"
#include <string>
#include <vector>
#include "nav_2d_utils/parameters.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"

PLUGINLIB_EXPORT_CLASS(dwb_critics::RotateToGoalCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

inline double hypot_sq(double dx, double dy)
{
  return dx * dx + dy * dy;
}

void RotateToGoalCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  xy_goal_tolerance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".xy_goal_tolerance", 0.25);
  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
  double stopped_xy_velocity = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + ".trans_stopped_velocity", 0.25);
  stopped_xy_velocity_sq_ = stopped_xy_velocity * stopped_xy_velocity;
  slowing_factor_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".slowing_factor", 5.0);
  lookahead_time_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".lookahead_time", -1.0);
  reset();
}

void RotateToGoalCritic::reset()
{
  in_window_ = false;
  rotating_ = false;
}

bool RotateToGoalCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D &)
{
  double dxy_sq = hypot_sq(pose.x - goal.x, pose.y - goal.y);
  in_window_ = in_window_ || dxy_sq <= xy_goal_tolerance_sq_;
  current_xy_speed_sq_ = hypot_sq(vel.x, vel.y);
  rotating_ = rotating_ || (in_window_ && current_xy_speed_sq_ <= stopped_xy_velocity_sq_);
  goal_yaw_ = goal.theta;
  return true;
}


/*--
dwb_local_planner 로부터 궤적 지원자를 아규먼트로 받는다.
궤적 지원자의 cost 를 할당하여 반환한다.
*/
double RotateToGoalCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{


  // If we're not sufficiently close to the goal, we don't care what the twist is

/*--
로봇이 최종 goal 지점과 특정 거리 이내에 있지 않으면 rotate_to_goal critic 은 동작하지 않는다.

'in_window' 는 (로봇 ~ 최종 goal 간의 거리) 가 'xy_goal_tolerance' ros_parameter 보다 작은지 아닌지로 결정된다.
(로봇 ~ 최종 goal 간의 거리) 가 'xy_goal_tolerance' 이내의 값이면 로봇이 최종 goal 지점에 충분히 가깝다고 판단한다.
*/
  if (!in_window_)
  {
    return 0.0;
  }


/*--
로봇이 최종 goal 지점에 근접해 있는 상태에서 로봇의 현재 선속도 (vx, vy) 가 아직 일정 값 이상이면, 평가할 궤적 지원자의 선속도 (vx, vy) 가 크면 클수록 높은 cost 를 할당한다.
즉, 이 경우에는 평가할 궤적 지원자의 선속도 (vx, vy) 가 낮을수록 낮은 cost 를 할당한다.

로봇의 현재 선속도 (vx, vy) 가 일정 값 이상인지 아닌지는 'rotating' 값으로 결정된다.
'in_window' 가 true 이며, (2 wheel differential model 인 경우) 로봇의 현재 vx 가 'stopped_xy_velocity_sq_' ros_parameter 보다 작을 경우 'rotating' 는 true 가 된다.
즉, 이 경우에는 로봇이 선속도가 굉장히 낮고 로봇이 최종 goal 지점에서 회전중인 상태라고 판단한다.
*/
  else if (!rotating_)
  {
    double speed_sq = hypot_sq(traj.velocity.x, traj.velocity.y);


/*--
로봇이 최종 goal 지점에 근접해 있는데도 평가할 궤적 지원자의 속도가 현재 로봇의 속도보다 커지는 경향이면 예외를 throw 한다.
*/
    if (speed_sq >= current_xy_speed_sq_)
    {
      throw dwb_core::IllegalTrajectoryException(name_, "Not slowing down near goal.");
    }


/*--
로봇이 최종 goal 지점에 근접해 있는 상태에서 로봇의 현재 선속도 (vx, vy) 가 아직 일정 값 이상이면, 평가할 궤적 지원자의 선속도 (vx, vy) 가 크면 클수록 높은 cost 를 할당한다.
즉, 이 경우에는 평가할 궤적 지원자의 선속도 (vx, vy) 가 낮을수록 낮은 cost 를 할당한다.
*/
    return speed_sq * slowing_factor_ + scoreRotation(traj);
  }


  // If we're sufficiently close to the goal, any transforming velocity is invalid

/*--
위의 if 문을 통과하고 여기에 도달했다는 것은, 로봇이 최종 goal 지점에 근접해 있으며 로봇의 선속도가 충분히 낮은 상태라는 것이다.
여기부터 로봇은 이제 주행은 멈추고 최종 goal 방향으로의 회전을 해야 한다.
따라서 평가할 궤적 지원자의 선속도가 0 보다 큰 경우는 예외를 throw 한다.

즉, 각속도만 있는 궤적 지원자만 평가되고 그 중 하나가 최적 궤적 지원자로 선정된다.
*/
  if (fabs(traj.velocity.x) > 0 || fabs(traj.velocity.y) > 0)
  {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Nonrotation command near goal.");
  }


  return scoreRotation(traj);
}


/*--

*/
double RotateToGoalCritic::scoreRotation(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (traj.poses.empty())
  {
    throw dwb_core::IllegalTrajectoryException(name_, "Empty trajectory.");
  }


  double end_yaw;


  if (lookahead_time_ >= 0.0)
  {
    geometry_msgs::msg::Pose2D eval_pose = dwb_core::projectPose(traj, lookahead_time_);

    end_yaw = eval_pose.theta;
  }
  else
  {
    end_yaw = traj.poses.back().theta;
  }


  return fabs(angles::shortest_angular_distance(end_yaw, goal_yaw_));
}

}  // namespace dwb_critics
