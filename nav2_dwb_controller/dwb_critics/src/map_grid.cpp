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

#include "dwb_critics/map_grid.hpp"
#include <cmath>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <memory>
#include "dwb_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

using std::abs;
using costmap_queue::CellData;

namespace dwb_critics
{

// Customization of the CostmapQueue validCellToQueue method
bool MapGridCritic::MapGridQueue::validCellToQueue(const costmap_queue::CellData & /*cell*/)
{
  return true;
}

void MapGridCritic::onInit()
{
  costmap_ = costmap_ros_->getCostmap();
  queue_ = std::make_shared<MapGridQueue>(*costmap_, *this);

  // Always set to true, but can be overriden by subclasses
  stop_on_failure_ = true;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".aggregation_type",
    rclcpp::ParameterValue(std::string("last")));

  std::string aggro_str;
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".aggregation_type", aggro_str);
  std::transform(aggro_str.begin(), aggro_str.end(), aggro_str.begin(), ::tolower);
  if (aggro_str == "last") {
    aggregationType_ = ScoreAggregationType::Last;
  } else if (aggro_str == "sum") {
    aggregationType_ = ScoreAggregationType::Sum;
  } else if (aggro_str == "product") {
    aggregationType_ = ScoreAggregationType::Product;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "MapGridCritic"), "aggregation_type parameter \"%s\" invalid. Using Last.",
      aggro_str.c_str());
    aggregationType_ = ScoreAggregationType::Last;
  }
}

void MapGridCritic::setAsObstacle(unsigned int index)
{
  cell_values_[index] = obstacle_score_;
}

void MapGridCritic::reset()
{
  queue_->reset();
  cell_values_.resize(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY());
  obstacle_score_ = static_cast<double>(cell_values_.size());
  unreachable_score_ = obstacle_score_ + 1.0;
  std::fill(cell_values_.begin(), cell_values_.end(), unreachable_score_);
}

void MapGridCritic::propogateManhattanDistances()
{
  while (!queue_->isEmpty()) {
    costmap_queue::CellData cell = queue_->getNextCell();
    cell_values_[cell.index_] = CellData::absolute_difference(cell.src_x_, cell.x_) +
      CellData::absolute_difference(cell.src_y_, cell.y_);
  }
}


/*--
dwb_local_planner 에 의해 호출된다.
특정 속도 벡터 (v, w) 에 해당하는 궤적을 아규먼트로 받는다.
map_grid critic 은 자신의 기준에 의거하여 해당 궤적에 대한 cost 를 평가하고 반환한다.
*/
double MapGridCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  double score = 0.0;
  unsigned int start_index = 0;

/*--
map_grid critic 이 궤적에 cost 를 평가하는 방법은 3가지가 존재한다.

1. Last: 해당 궤적의 마지막 pose 만 체크하여 cost 를 할당한다.
2. Sum: 해당 궤적의 모든 pose 의 cost 를 합산하여 최종 cost 로 할당한다.
3. Product: 해당 궤적의 모든 pose 의 cost 중, 0 이상인 값만 모두 곱하여 최종 cost 로 할당한다.
*/
  if (aggregationType_ == ScoreAggregationType::Product)
  {
    score = 1.0;
  }


/*--
stop_on_failure 가 true인 경우,
map_grid critic 은 평가 중인 궤적의 특정 pose가 장애물 위에 있거나
도달할 수 없는 영역 위에 있을 경우 해당 궤적의 평가를 중단하고 예외를 throw 한다.
map_grid critic 에서 stop_on_failure 은 default 로 true 이다.

aggregation 타입이 Last 이며,
1. stop_on_failure 가 false인 경우 map_grid critic 은 해당 path 의 마지막 pose 만 체크하면 된다.
2. stop_on_failure 가 true인 경우 map_grid critic 은 모든 pose 가 장애물 위에 있는지 또는 도달할 수 없는 영역 위에 있는지 체크해야 하므로
    모든 pose 를 체크해야 한다.
*/
  else if (aggregationType_ == ScoreAggregationType::Last && !stop_on_failure_)
  {
    start_index = traj.poses.size() - 1;
  }


  double grid_dist;


/*--
궤적의 각 pose를 순회하며 cost 를 평가한다.
aggregation 타입에 따라 최종 cost 를 평가한다.
*/
  for (unsigned int i = start_index; i < traj.poses.size(); ++i)
  {


/*--
아규먼트로 궤적의 특정 pose 를 넘겨둔다.
특정 pose ~ 사전에 지정된 최적 지향 pose 까지의 거리가 반환된다.

이때, 거리는 맨하탄 거리 또는 유클리드 거리로 계산된다.
최적 지향 pose 로부터의 거리가 멀수록 cost 는 커진다.
*/
    grid_dist = scorePose(traj.poses[i]);


/*--
stop_on_failure 가 true 인 경우, 최적 지향 pose ~ 특정 pose 까지의 거리가 장애물 까지의 거리인지 또는 도달할 수 없는 거리인지 체크한다.
*/
    if (stop_on_failure_)
    {
      if (grid_dist == obstacle_score_)
      {
        throw dwb_core::
              IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
      }
      else if (grid_dist == unreachable_score_)
      {
        throw dwb_core::
              IllegalTrajectoryException(name_, "Trajectory Hits Unreachable Area.");
      }
    }


/*--
aggregation 타입에 따라 최종 cost 를 할당한다.
*/
    switch (aggregationType_)
    {
      case ScoreAggregationType::Last:
        score = grid_dist;
        break;
      case ScoreAggregationType::Sum:
        score += grid_dist;
        break;
      case ScoreAggregationType::Product:
        if (score > 0)
        {
          score *= grid_dist;
        }
        break;
    }
  }


  return score;
}


/*--
아규먼트로 궤적의 특정 pose 를 받는다.
특정 pose ~ 사전에 지정된 최적 지향 pose 까지의 거리를 반환한다.

이때, 거리는 맨하탄 거리 또는 유클리드 거리로 계산된다.
*/
double MapGridCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  unsigned int cell_x, cell_y;


// we won't allow trajectories that go off the map... shouldn't happen that often anyways

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
특정 cell ~ 사전에 지정된 최적 지향 cell 까지의 거리가 반환된다.

이때, 거리는 맨하탄 거리 또는 유클리드 거리로 계산된다.
*/
  return getScore(cell_x, cell_y);
}

void MapGridCritic::addCriticVisualization(
  std::vector<std::pair<std::string, std::vector<float>>> & cost_channels)
{
  std::pair<std::string, std::vector<float>> grid_scores;
  grid_scores.first = name_;

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();
  grid_scores.second.resize(size_x * size_y);
  unsigned int i = 0;
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
      grid_scores.second[i] = getScore(cx, cy);
      i++;
    }
  }
  cost_channels.push_back(grid_scores);
}

}  // namespace dwb_critics
