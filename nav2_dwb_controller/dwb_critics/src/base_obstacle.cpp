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
#include <vector>
#include <string>
#include <utility>

#include "dwb_critics/base_obstacle.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"


PLUGINLIB_EXPORT_CLASS(dwb_critics::BaseObstacleCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

void BaseObstacleCritic::onInit()
{
  costmap_ = costmap_ros_->getCostmap();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".sum_scores", rclcpp::ParameterValue(false));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".sum_scores", sum_scores_);
}


/*--
dwb_local_planner 로부터 궤적 지원자를 아규먼트로 받는다.
궤적 지원자의 cost 를 할당하여 반환한다.
*/
double BaseObstacleCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  double score = 0.0;


/*--
궤적 지원자의 각 pose 개수만큼 반복한다.
각각의 pose가 costmap 상에서 장애물 위에 있으면 높은 cost 를 할당하고,
free space 위에 있으면 낮은 cost 를 할당한다.

만약, 'sum_scores' 파라미터가 true 이면 해당 궤적 지원자의 모든 pose 의 점수를 합산하여 반환한다.
false 이면 해당 궤적 지원자의 마지막 pose 의 점수만 반환한다.

(수정함) =>
false 이면, 궤적 지원자의 모든 pose 중 가장 높은 cost 가 최종 cost 로 할당된다.
*/
  for (unsigned int i = 0; i < traj.poses.size(); ++i) {


/*--
궤적 지원자의 각 pose 를 아규먼트로 넣어준다.
해당 pose 의 cost 가 반환된다.
*/
    double pose_score = scorePose(traj.poses[i]);


    // Optimized/branchless version of if (sum_scores_) score += pose_score,
    // else score = pose_score;

/*--
만약, sum_scores 파라미터가 true이면 해당 궤적 지원자의 모든 pose의 점수가 합산된다.
false 이면 score에는 최종적으로 해당 궤적 지원자의 마지막 pose의 점수만 들어간다.

(수정함) =>
false 이면, 궤적 지원자의 모든 pose 중 가장 높은 cost 가 최종 cost 로 할당된다.
*/
// score = static_cast<double>(sum_scores_) * score + pose_score;
    if (score < static_cast<double>(sum_scores_) * score + pose_score)
    {
      score = static_cast<double>(sum_scores_) * score + pose_score;
    }
  }


  return score;
}


/*--
궤적 지원자의 각 pose 를 아규먼트로 받는다.
해당 pose 의 cost 를 반환한다.
*/
double BaseObstacleCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  unsigned int cell_x, cell_y;


/*--
아규먼트로 map frame 을 원점으로 하는 좌표계 에서의 pose 좌표 값 (pose.x, pose.y) 을 넣어준다.
(cell_x, cell_y) 에는 지도의 왼쪽 아래 점을 원점으로 하는 지도 좌표계 에서의 좌표가 들어가게 된다.
이때, (cell_x, cell_y) 는 grid 단위의 좌표로 표현된다.
*/
  if (!costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y))
  {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  }


/*--
아규먼트로 지도 왼쪽 아래 점 좌표계에서의 grid 좌표를 넣어준다.
현재 costmap 에서 해당 좌표에서의 cost 값이 반환된다.
*/
  unsigned char cost = costmap_->getCost(cell_x, cell_y);


  if (!isValidCost(cost))
  {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
  }


  return cost;
}


bool BaseObstacleCritic::isValidCost(const unsigned char cost)
{
  return cost != nav2_costmap_2d::LETHAL_OBSTACLE &&
         cost != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
         cost != nav2_costmap_2d::NO_INFORMATION;
}

void BaseObstacleCritic::addCriticVisualization(
  std::vector<std::pair<std::string, std::vector<float>>> & cost_channels)
{
  std::pair<std::string, std::vector<float>> grid_scores;
  grid_scores.first = name_;

  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  grid_scores.second.resize(size_x * size_y);
  unsigned int i = 0;
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
      grid_scores.second[i] = costmap_->getCost(cx, cy);
      i++;
    }
  }
  cost_channels.push_back(grid_scores);
}

}  // namespace dwb_critics
