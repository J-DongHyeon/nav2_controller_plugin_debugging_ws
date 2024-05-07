# nav2 controller plugin 분석

<br>

#### nav2_dwb_controller

<br>
---

method 호출 순서
- controller_server: cs
- dwb_local_planner: dlp
- standard_traj_generator: stg
- xy_theta_iterator: xti
- one_d_velocity_iterator: ovi

- (cs) computeAndPublishVelocity() -> (cs) getThresholdedTwist() -> (dlp) computeVelocityCommands() -> (dlp) computeVelocityCommands() -> (dlp) coreScoringAlgorithm() -> (stg) startNewIteration() -> (xti) startNewIteration() -> (stg) hasMoreTwists() -> (xti) hasMoreTwists() -> (ovi) isFinished() -> (stg) nextTwist() -> (xti) nextTwist() -> (ovi) getVelocity() -> (xti) iterateToValidVelocity() -> (ovi) operator++ -> (stg) generateTrajectory() -> (stg) getTimeSteps() -> (stg) computeNewVelocity() -> (stg) computeNewPosition() -> (dlp) scoreTrajectory() -> (각 critic) scoreTrajectory() -> (cs) publishVelocity()


critic 분석 순서
- map_grid critic (mg)
    (mg) scoreTrajectory() -> (mg) scorePose() -> (mg) getScore()

    => map_grid critic 은 특정 속도 벡터 (v, w) 에 대한 궤적의 각 pose 에 대해, (사전에 지정된 pose) ~ (궤적의 각 pose) 까지의 거리가 가깝냐에 따라 cost 를 평가한다.
    이때, 거리는 맨하탄 거리 또는 유클리드 거리로 계산된다.


- goal_dist critic (gd)
    (mg) scoreTrajectory() -> (mg) scorePose() -> (mg) getScore()

    => goal_dist critic 은 map_grid critic 을 상속 받으며, 궤적 cost 평가 원리가 map_grid critic 과 동일하다.
    이때, 사전에 지정된 pose 는 로봇의 local_cost_map 에서 로봇과 가장 멀리 떨어져 있는 global path 의 pose 가 된다.

    즉, 궤적의 각 pose 에 대해 (local_cost_map 에서 로봇과 가장 거리가 먼 global path 의 pose) ~ (궤적의 각 pose) 까지의 거리가 가깝냐에 따라 cost 가 평가된다.


- goal_align critic (ga)
    (mg) scoreTrajectory() -> (ga) scorePose() -> (mg) getScore()

    => goal_align critic 은 goal_dist critic 을 상속 받으며, 궤적 cost 평가 원리가 goal_dist critic 과 동일하다.
    goal_align critic 은 궤적의 각 pose 에 대해 cost 를 평가할 때, 해당 pose 가 아닌 해당 pose 로부터 'forward_point_distance' 만큼 전진한 pose 에 대해 cost 평가를 한다.
    따라서 만약 해당 pose 의 angle 이 (사전에 지정된 최적 지향 지점) 에 정렬 되있는 상태이면 두 지점간의 거리가 가까워 지므로 낮은 cost 평가를 받고, 정렬 되있지 않은 상태이면 높은 cost 평가를 받는다.

    즉, goal_align critic 은 궤적의 각 pose 가 (사전에 지정된 최적 지향 지점) 에 잘 정렬 되있을수록 낮은 cost 평가를 준다.


- path_dist critic (pd)
    (mg) scoreTrajectory() -> (mg) scorePose() -> (mg) getScore()

    => path_dist critic 은 map_grid critic 을 상속 받으며, 궤적 cost 평가 원리가 map_grid critic 과 동일하다.
    이때, 사전에 지정된 pose 는 로봇의 local_cost_map 에서 로봇과 가장 가까운 global path 의 pose 가 된다.

    즉, 궤적의 각 pose 에 대해 (local_cost_map 에서 로봇과 가장 가까운 global path 의 pose) ~ (궤적의 각 pose) 까지의 거리가 가깝냐에 따라 cost 가 평가된다.


- path_align critic (pa)
    (mg) scoreTrajectory() -> (pa) scorePose() -> (mg) getScore()

    => path_align critic 은 path_dist critic 을 상속 받으며, 궤적 cost 평가 원리가 path_dist critic 과 동일하다.
    path_align critic 은 궤적의 각 pose 에 대해 cost 를 평가할 때, 해당 pose 가 아닌 해당 pose 로부터 'forward_point_distance' 만큼 전진한 pose 에 대해 cost 평가를 한다.
    따라서 만약 해당 pose 의 angle 이 (사전에 지정된 최적 지향 지점) 에 정렬 되있는 상태이면 두 지점간의 거리가 가까워 지므로 낮은 cost 평가를 받고, 정렬 되있지 않은 상태이면 높은 cost 평가를 받는다.

    즉, path_align critic 은 궤적의 각 pose 가 (사전에 지정된 최적 지향 지점) 에 잘 정렬 되있을수록 낮은 cost 평가를 준다.


- base_obstacle critic (bo)
    (bo) scoreTrajectory() -> (bo) scorePose()

    => base_obstacle critic 은 궤적의 각 pose 가 costmap 상에서 free space 에 있는지, 장애물 영역에 있는지를 체크한다.
    만약, 궤적의 각 pose 가 free space 에 있으면 낮은 cost 를 할당하고, 장애물 영역에 있으면 높은 cost 를 할당한다.

    이때, ‘sum_scores’ 파라미터가 true 이면, 궤적 지원자의 모든 pose 에서의 점수를 합산하여 cost 를 할당한다.
    false 이면, 궤적 지원자의 마지막 pose 에서의 점수를 cost 로 할당한다.
    (수정함) =>
    false 이면, 궤적 지원자의 모든 pose 중 가장 높은 cost 가 최종 cost 로 할당된다.


- rotate_to_goal critic (rog)
    (rog) scoreTrajectory() -> (rog) scoreRotation()
    => 
