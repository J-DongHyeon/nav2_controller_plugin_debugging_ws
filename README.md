# nav2 controller plugin 분석

<br>

#### nav2_dwb_controller

method 호출 순서
- controller_server: cs
- dwb_local_planner: dlp
- standard_traj_generator: stg
- xy_theta_iterator: xti
- one_d_velocity_iterator: ovi

- (cs) computeAndPublishVelocity() -> (cs) getThresholdedTwist() -> (dlp) computeVelocityCommands() -> (dlp) computeVelocityCommands() -> (dlp) coreScoringAlgorithm() -> (stg) startNewIteration() -> (xti) startNewIteration() -> (stg) hasMoreTwists() -> (xti) hasMoreTwists() -> (ovi) isFinished() -> (stg) nextTwist() -> (xti) nextTwist() -> (ovi) getVelocity() -> (xti) iterateToValidVelocity() -> (ovi) operator++ -> (stg) generateTrajectory() -> (stg) getTimeSteps() -> (stg) computeNewVelocity() -> (stg) computeNewPosition() -> (dlp) scoreTrajectory() -> (각 critic) scoreTrajectory() -> (cs) publishVelocity()

