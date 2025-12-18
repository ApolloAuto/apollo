bool Planning::FillPlanningResult(
    const apollo::planning::Frame& frame,
    const std::vector<apollo::common::TrajectoryPoint>& stitching_trajectory,
    bestway::pnc::planning::ADCTrajectory* const planning_result) {

  if (frame.reference_line_info().empty()) {
    BERROR << "No reference line info in frame.";
    return false;
  }

  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& trajectory = reference_line_info.trajectory();
  RETURN_VAL_IF(trajectory.empty(), false);

  BINFO << "Final traj length: " << trajectory.GetSpatialLength()
        << ", path length: " << reference_line_info.path_data().discretized_path().Length();

  VehicleSignal::TurnSignal desired_turn_signal = VehicleSignal::TURN_NONE;

  constexpr double kMaxLookaheadDist = 10.0;      // 沿参考线看10米
  constexpr double kMinTurnIntensity = 0.2;       // 总曲率强度阈值
  constexpr double kDominanceRatio = 1.2;         // 主导方向需强20%

  if (!trajectory.empty()) {
    const auto& reference_line = reference_line_info.reference_line();

    const auto& ego_path_point = trajectory.front().path_point();
    apollo::common::SLPoint ego_sl;
    if (!reference_line.XYToSL({ego_path_point.x(), ego_path_point.y()}, &ego_sl)) {
      BERROR << "Failed to project ego pose to reference line for turn signal logic.";
    } else {
      double ego_s = ego_sl.s();
      double left_turn_intensity = 0.0;
      double right_turn_intensity = 0.0;
      bool has_valid_kappa = false;
      double prev_s = ego_s;  // 上一个有效点的 s

      for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& pt = trajectory[i];
        if (!pt.path_point().has_kappa()) continue;

        apollo::common::SLPoint curr_sl;
        if (!reference_line.XYToSL({pt.path_point().x(), pt.path_point().y()}, &curr_sl)) {
          continue; 
        }

        double curr_s = curr_sl.s();
        // 忽略后方点
        if (curr_s < ego_s) continue;

        // 超出预瞄距离，停止
        if (curr_s - ego_s > kMaxLookaheadDist) break;

        double ds = curr_s - prev_s;
        if (ds < 1e-3) {
          // 点太近，不积分，但更新 prev_s 防止累积误差
          prev_s = curr_s;
          continue;
        }

        double kappa = pt.path_point().kappa();
        if (kappa > 0) {
          left_turn_intensity += kappa * ds;
        } else if (kappa < 0) {
          right_turn_intensity += (-kappa) * ds;
        }

        prev_s = curr_s;
        has_valid_kappa = true;
      }

      // 判断是否打灯
      if (has_valid_kappa) {
        double total_intensity = left_turn_intensity + right_turn_intensity;
        if (total_intensity >= kMinTurnIntensity) {
          if (left_turn_intensity > right_turn_intensity * kDominanceRatio) {
            desired_turn_signal = VehicleSignal::TURN_LEFT;
          } else if (right_turn_intensity > left_turn_intensity * kDominanceRatio) {
            desired_turn_signal = VehicleSignal::TURN_RIGHT;
          }
        }
      }
    }
  }

  // --- 接近目标点时取消转向灯 ---
  double delta_s_to_goal = std::numeric_limits<double>::max();
  const auto& goal = frame.goal();
  const auto& reference_line = frame.reference_line_info().front().reference_line();
  const auto& adc_pose = trajectory.front().path_point();

  apollo::common::SLPoint adc_sl, goal_sl;
  if (reference_line.XYToSL({adc_pose.x(), adc_pose.y()}, &adc_sl) &&
      reference_line.XYToSL({goal.x(), goal.y()}, &goal_sl)) {
    delta_s_to_goal = goal_sl.s() - adc_sl.s();
  }
  BERROR << "delta_s_to_goal: " << delta_s_to_goal;

  constexpr double kTurnSignalCancelDistAlongS = 8.0;
  if (delta_s_to_goal > 0 && delta_s_to_goal < kTurnSignalCancelDistAlongS) {
    BERROR << "Near goal (Δs=" << delta_s_to_goal << "m), turn signal canceled.";
    desired_turn_signal = VehicleSignal::TURN_NONE;
  }

  for (size_t i = 0; i < stitching_trajectory.size(); ++i) {
    auto* point = planning_result->add_trajectory_point();
    ConvertApolloToBestwayPoint(stitching_trajectory[i], point);
  }

  for (const auto& apollo_point : trajectory) {
    auto* point = planning_result->add_trajectory_point();
    ConvertApolloToBestwayPoint(apollo_point, point);
  }

  switch (desired_turn_signal) {
    case VehicleSignal::TURN_LEFT:
      planning_result->set_turn_signal(bestway::pnc::TurnSignal::TURN_LEFT);
      break;
    case VehicleSignal::TURN_RIGHT:
      planning_result->set_turn_signal(bestway::pnc::TurnSignal::TURN_RIGHT);
      break;
    default:
      planning_result->set_turn_signal(bestway::pnc::TurnSignal::TURN_NONE);
      break;
  }

  planning_result->set_gear(bestway::pnc::GearPosition::GEAR_D);
  return true;
}