/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file behavior_planner_FSM.cpp
 **/

#include "behavior_planner_FSM.h"

State BehaviorPlannerFSM::get_closest_waypoint_goal(
    const State& ego_state, const SharedPtr<cc::Map>& map,
    const float& lookahead_distance, bool& is_goal_junction)
{
  // Nearest waypoint on the center of a Driving Lane.
  auto waypoint_0 = map->GetWaypoint(ego_state.location);

  if (_active_maneuver == DECEL_TO_STOP || _active_maneuver == STOPPED)
  {
    State waypoint;

    auto wp_transform = waypoint_0->GetTransform();
    waypoint.location = wp_transform.location;
    waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
    waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
    waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);

    return waypoint;
  }

  // Waypoints at a lookahead distance
  // NOTE: "GetNext(d)" creates a list of waypoints at an approximate distance
  // "d" in the direction of the lane. The list contains one waypoint for each
  // deviation possible.

  // NOTE 2: GetNextUntilLaneEnd(d) returns a list of waypoints a distance "d"
  // apart. The list goes from the current waypoint to the end of its
  // lane.

  auto lookahead_waypoints = waypoint_0->GetNext(lookahead_distance);

  auto n_wp = lookahead_waypoints.size();
  if (n_wp == 0)
  {
    State waypoint;
    return waypoint;
  }

  waypoint_0 = lookahead_waypoints[lookahead_waypoints.size() - 1];

  is_goal_junction = waypoint_0->IsJunction();

  auto cur_junction_id = waypoint_0->GetJunctionId();

  if (is_goal_junction)
  {
    if (cur_junction_id == _prev_junction_id)
    {
      is_goal_junction = false;
    }
    else
    {
      _prev_junction_id = cur_junction_id;
    }
  }

  State waypoint;

  auto wp_transform = waypoint_0->GetTransform();
  waypoint.location = wp_transform.location;
  waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
  waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
  waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);

  return waypoint;
}

double BehaviorPlannerFSM::get_look_ahead_distance(const State& ego_state) {
  auto velocity_mag = utils::magnitude(ego_state.velocity);
  auto accel_mag = utils::magnitude(ego_state.acceleration);

  // distance needed to come to a stop while traveling at speed V and
  // using a comfortable deceleration.
  auto look_ahead_distance = (velocity_mag*velocity_mag) / (2*accel_mag);

  look_ahead_distance =
      std::min(std::max(look_ahead_distance, _lookahead_distance_min),
               _lookahead_distance_max);
  return look_ahead_distance;
}

State BehaviorPlannerFSM::get_goal(const State& ego_state,
                                   SharedPtr<cc::Map> map)
{
  // Get look-ahead distance based on Ego speed
  auto look_ahead_distance = get_look_ahead_distance(ego_state);

  // Nearest waypoint on the center of a Driving Lane.
  bool is_goal_in_junction{false};
  auto goal_wp = get_closest_waypoint_goal(ego_state, map, look_ahead_distance,
                                           is_goal_in_junction);

  string tl_state = "none";
  State goal =
      state_transition(ego_state, goal_wp, is_goal_in_junction, tl_state);

  return goal;
}

State BehaviorPlannerFSM::state_transition(const State& ego_state, State goal,
                                           bool& is_goal_in_junction,
                                           string tl_state) {

  // Check with the Behavior Planner to see what we are going to do and
  // where our next goal is
  goal.acceleration.x = 0;
  goal.acceleration.y = 0;
  goal.acceleration.z = 0;

  if (_active_maneuver == FOLLOW_LANE)
  {
    if (is_goal_in_junction)
    {
      _active_maneuver = DECEL_TO_STOP;
      // Let's backup a "buffer" distance behind the "STOP" point

      // goal behind the stopping point: put the goal behind the stopping
      // point (i.e the actual goal location) by "_stop_line_buffer". HINTS:
      // remember that we need to go back in the opposite direction of the
      // goal/road, i.e you should use: ang = goal.rotation.yaw + M_PI and then
      // use cosine and sine to get x and y
      auto ang = goal.rotation.yaw + M_PI;
      goal.location.x += cos(ang) + _stop_line_buffer;
      goal.location.y += sin(ang) + _stop_line_buffer;

      // Goal speed at stopping point: What should be the goal speed??
      goal.velocity.x = 0.0;
      goal.velocity.y = 0.0;
      goal.velocity.z = 0.0;

    }
    else
    {
      // Goal speed in nominal state i.e. not a junction
      goal.velocity.x = _speed_limit * std::cos(goal.rotation.yaw);
      goal.velocity.y = _speed_limit * std::sin(goal.rotation.yaw);
      goal.velocity.z = 0.0;
    }

  }
  else if (_active_maneuver == DECEL_TO_STOP)
  {
    // Maintain the same goal when in DECEL_TO_STOP state: Make sure the
    // new goal is the same as the previous goal (_goal). That way we
    // keep/maintain the goal at the stop line.
    goal = _goal;

    // It turns out that when we teleport, the car is always at speed
    // zero. In this the case, as soon as we enter the DECEL_TO_STOP state,
    // the condition that we are <= _stop_threshold_speed is ALWAYS true and we
    // move straight to "STOPPED" state. To solve this issue (since we don't
    // have a motion controller yet), you should use "distance" instead of
    // speed. Make sure the distance to the stopping point is <=
    // P_STOP_THRESHOLD_DISTANCE. Uncomment the line used to calculate the
    // distance
    auto distance_to_stop_sign =
        utils::magnitude(goal.location - ego_state.location);

    // Use distance rather than speed
    //if (utils::magnitude(ego_state.velocity) <=
    //    _stop_threshold_speed) {  // -> Fix this
    if (distance_to_stop_sign <= P_STOP_THRESHOLD_DISTANCE)
    {
        // TODO-move to STOPPED state: Now that we know we are close or at the
        // stopping point we should change state to "STOPPED"
        _active_maneuver = STOPPED;  // <- Fix This
        _start_stop_time = std::chrono::high_resolution_clock::now();
    }

  }
  else if (_active_maneuver == STOPPED)
  {
    // Maintain the same goal when in STOPPED state: Make sure the new goal
    // is the same as the previous goal. That way we keep/maintain the goal at
    // the stop line.
    goal = _goal;

    long long stopped_secs =
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::high_resolution_clock::now() - _start_stop_time)
            .count();

    if (stopped_secs >= _req_stop_time && tl_state.compare("Red") != 0)
    {
      // Move to FOLLOW_LANE state: What state do we want to move to, when
      // we are "done" at the STOPPED state?
      _active_maneuver = FOLLOW_LANE;
    }
  }
  _goal = goal;
  return goal;
}
