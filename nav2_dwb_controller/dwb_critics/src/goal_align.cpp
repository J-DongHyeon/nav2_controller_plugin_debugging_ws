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

#include "dwb_critics/goal_align.hpp"
#include <vector>
#include <string>
#include "dwb_critics/alignment_util.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/parameters.hpp"

namespace dwb_critics
{

void GoalAlignCritic::onInit()
{
  GoalDistCritic::onInit();
  stop_on_failure_ = false;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  forward_point_distance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".forward_point_distance", 0.325);
}

bool GoalAlignCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  // we want the robot nose to be drawn to its final position
  // (before robot turns towards goal orientation), not the end of the
  // path for the robot center. Choosing the final position after
  // turning towards goal orientation causes instability when the
  // robot needs to make a 180 degree turn at the end
  double angle_to_goal = atan2(goal.y - pose.y, goal.x - pose.x);

  nav_2d_msgs::msg::Path2D target_poses = global_plan;
  target_poses.poses.back().x += forward_point_distance_ * cos(angle_to_goal);
  target_poses.poses.back().y += forward_point_distance_ * sin(angle_to_goal);

  return GoalDistCritic::prepare(pose, vel, goal, target_poses);
}


/*--
goal_align critic 은 궤적의 각 pose 에 대해 cost 를 평가할 때, 해당 pose 가 아닌 해당 pose 로부터 'forward_point_distance' 만큼 전진한 pose 에 대해 cost 평가를 한다.
따라서 만약 해당 pose 의 angle 이 (사전에 지정된 최적 지향 지점) 에 정렬 되있는 상태이면 두 지점간의 거리가 가까워 지므로 낮은 cost 평가를 받고, 정렬 되있지 않은 상태이면 높은 cost 평가를 받는다.

즉, goal_align critic 은 궤적의 각 pose 가 (사전에 지정된 최적 지향 지점) 에 잘 정렬 되있을수록 낮은 cost 평가를 준다.
*/
double GoalAlignCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{


/*--
궤적의 각 pose 에 대해 'forward_point_distance' 만큼 전진한 pose 를 cost 평가한다.
*/
  return GoalDistCritic::scorePose(getForwardPose(pose, forward_point_distance_));
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::GoalAlignCritic, dwb_core::TrajectoryCritic)
