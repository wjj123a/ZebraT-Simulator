#include <zebrat/hybrid_a_star_global_planner.h>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace zebrat
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

HybridAStarGlobalPlanner::HybridAStarGlobalPlanner() = default;

HybridAStarGlobalPlanner::HybridAStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(std::move(name), costmap_ros);
}

void HybridAStarGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("HybridAStarGlobalPlanner has already been initialized");
    return;
  }

  if (costmap_ros == nullptr)
  {
    ROS_ERROR("HybridAStarGlobalPlanner requires a valid Costmap2DROS");
    return;
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  world_model_.reset(new base_local_planner::CostmapModel(*costmap_));
  global_frame_ = costmap_ros_->getGlobalFrameID();

  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("minimum_turning_radius", minimum_turning_radius_, minimum_turning_radius_);
  private_nh.param("step_size", step_size_, step_size_);
  private_nh.param("yaw_bins", yaw_bins_, yaw_bins_);
  private_nh.param("max_iterations", max_iterations_, max_iterations_);
  private_nh.param("allow_reverse", allow_reverse_, allow_reverse_);
  private_nh.param("allow_unknown", allow_unknown_, allow_unknown_);
  int lethal_cost = lethal_cost_;
  private_nh.param("lethal_cost", lethal_cost, lethal_cost);
  lethal_cost_ = static_cast<unsigned char>(std::max(0, std::min(255, lethal_cost)));
  private_nh.param("neutral_cost", neutral_cost_, neutral_cost_);
  private_nh.param("cost_factor", cost_factor_, cost_factor_);
  private_nh.param("reverse_penalty", reverse_penalty_, reverse_penalty_);
  private_nh.param("direction_change_penalty", direction_change_penalty_, direction_change_penalty_);
  private_nh.param("steering_change_penalty", steering_change_penalty_, steering_change_penalty_);
  private_nh.param("goal_xy_tolerance", goal_xy_tolerance_, goal_xy_tolerance_);
  private_nh.param("goal_yaw_tolerance", goal_yaw_tolerance_, goal_yaw_tolerance_);
  private_nh.param("analytic_expansion_max_length", analytic_expansion_max_length_, analytic_expansion_max_length_);
  private_nh.param("footprint_padding", footprint_padding_, footprint_padding_);
  private_nh.param("publish_plan", publish_plan_, publish_plan_);

  minimum_turning_radius_ = std::max(0.05, minimum_turning_radius_);
  step_size_ = std::max(0.02, step_size_);
  yaw_bins_ = std::max(8, yaw_bins_);
  max_iterations_ = std::max(1, max_iterations_);
  neutral_cost_ = std::max(0.0, neutral_cost_);
  cost_factor_ = std::max(0.0, cost_factor_);
  reverse_penalty_ = std::max(1.0, reverse_penalty_);
  direction_change_penalty_ = std::max(0.0, direction_change_penalty_);
  steering_change_penalty_ = std::max(0.0, steering_change_penalty_);
  goal_xy_tolerance_ = std::max(0.02, goal_xy_tolerance_);
  goal_yaw_tolerance_ = std::max(0.02, goal_yaw_tolerance_);
  analytic_expansion_max_length_ = std::max(0.0, analytic_expansion_max_length_);
  footprint_padding_ = std::max(0.0, footprint_padding_);

  footprint_ = costmap_ros_->getRobotFootprint();
  if (footprint_.empty())
  {
    geometry_msgs::Point point;
    point.x = 0.30 + footprint_padding_;
    point.y = 0.18 + footprint_padding_;
    footprint_.push_back(point);
    point.x = 0.30 + footprint_padding_;
    point.y = -0.18 - footprint_padding_;
    footprint_.push_back(point);
    point.x = -0.30 - footprint_padding_;
    point.y = -0.18 - footprint_padding_;
    footprint_.push_back(point);
    point.x = -0.30 - footprint_padding_;
    point.y = 0.18 + footprint_padding_;
    footprint_.push_back(point);
  }

  plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1, true);
  initialized_ = true;

  ROS_INFO("Initialized Hybrid A* global planner with %.2fm turn radius, %.2fm step, %d yaw bins",
           minimum_turning_radius_,
           step_size_,
           yaw_bins_);
}

bool HybridAStarGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                        const geometry_msgs::PoseStamped& goal,
                                        std::vector<geometry_msgs::PoseStamped>& plan)
{
  double cost = 0.0;
  return makePlan(start, goal, plan, cost);
}

bool HybridAStarGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                        const geometry_msgs::PoseStamped& goal,
                                        std::vector<geometry_msgs::PoseStamped>& plan,
                                        double& cost)
{
  plan.clear();
  cost = 0.0;

  if (!initialized_ || costmap_ == nullptr)
  {
    ROS_ERROR("HybridAStarGlobalPlanner has not been initialized");
    return false;
  }

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_)
  {
    ROS_ERROR("Hybrid A* planner expects start and goal in %s, got %s and %s",
              global_frame_.c_str(),
              start.header.frame_id.c_str(),
              goal.header.frame_id.c_str());
    return false;
  }

  std::vector<State> states;
  int goal_state_index = -1;
  bool ok = false;
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    ok = planHybridAStar(start, goal, states, goal_state_index, cost);
  }

  if (!ok)
  {
    publishPlan(plan);
    return false;
  }

  buildPlan(states, goal_state_index, start, goal, plan);
  publishPlan(plan);
  return !plan.empty();
}

bool HybridAStarGlobalPlanner::planHybridAStar(const geometry_msgs::PoseStamped& start,
                                               const geometry_msgs::PoseStamped& goal,
                                               std::vector<State>& states,
                                               int& goal_state_index,
                                               double& path_cost)
{
  if (!isStateValid(State{start.pose.position.x, start.pose.position.y, yawFromPose(start)}))
  {
    ROS_WARN("Hybrid A* start pose is not valid");
    return false;
  }

  if (!isStateValid(State{goal.pose.position.x, goal.pose.position.y, yawFromPose(goal)}))
  {
    ROS_WARN("Hybrid A* goal pose is not valid");
    return false;
  }

  std::unordered_map<std::uint64_t, double> best_g;
  std::unordered_map<std::uint64_t, int> best_state;
  best_g.reserve(static_cast<std::size_t>(max_iterations_));
  best_state.reserve(static_cast<std::size_t>(max_iterations_));

  State start_state;
  start_state.x = start.pose.position.x;
  start_state.y = start.pose.position.y;
  start_state.yaw = yawFromPose(start);
  start_state.g = 0.0;
  start_state.parent = -1;
  start_state.reverse = false;

  states.push_back(start_state);
  StateKey start_key = stateKey(start_state);
  const std::uint64_t start_index = stateKeyIndex(start_key);
  best_g[start_index] = 0.0;
  best_state[start_index] = 0;

  std::priority_queue<NodeRecord, std::vector<NodeRecord>, NodeRecordCompare> open;
  open.push(NodeRecord{heuristic(start_state, goal), 0});

  const double goal_yaw = yawFromPose(goal);
  int iterations = 0;
  while (!open.empty() && iterations < max_iterations_)
  {
    const NodeRecord record = open.top();
    open.pop();
    ++iterations;

    const State current = states[record.state_index];
    const StateKey current_key = stateKey(current);
    const std::uint64_t current_key_index = stateKeyIndex(current_key);
    auto current_best_state = best_state.find(current_key_index);
    if (current_best_state == best_state.end() || record.state_index != current_best_state->second)
    {
      continue;
    }

    const double goal_distance = std::hypot(current.x - goal.pose.position.x, current.y - goal.pose.position.y);
    if (goal_distance <= analytic_expansion_max_length_ &&
        std::abs(shortestAngularDistance(current.yaw, goal_yaw)) <= goal_yaw_tolerance_)
    {
      State goal_state;
      goal_state.x = goal.pose.position.x;
      goal_state.y = goal.pose.position.y;
      goal_state.yaw = goal_yaw;
      goal_state.parent = record.state_index;
      goal_state.reverse = current.reverse;
      if (traversedMotionIsValid(current, goal_state, goal_state.g))
      {
        goal_state.g += current.g;
        states.push_back(goal_state);
        goal_state_index = static_cast<int>(states.size()) - 1;
        path_cost = goal_state.g;
        return true;
      }
    }

    if (isGoalState(current, goal))
    {
      goal_state_index = record.state_index;
      path_cost = current.g;
      return true;
    }

    for (State next : successors(current))
    {
      double motion_cost = 0.0;
      if (!traversedMotionIsValid(current, next, motion_cost))
      {
        continue;
      }

      next.parent = record.state_index;
      next.g = current.g + motion_cost + next.penalty;
      if (next.reverse)
      {
        next.g += step_size_ * (reverse_penalty_ - 1.0);
      }
      if (next.reverse != current.reverse)
      {
        next.g += direction_change_penalty_;
      }

      const StateKey next_key = stateKey(next);
      if (next_key.x < 0 || next_key.y < 0)
      {
        continue;
      }
      const std::uint64_t next_key_index = stateKeyIndex(next_key);
      auto known_g = best_g.find(next_key_index);
      if (known_g != best_g.end() && next.g >= known_g->second)
      {
        continue;
      }

      best_g[next_key_index] = next.g;
      states.push_back(next);
      const int next_state_index = static_cast<int>(states.size()) - 1;
      best_state[next_key_index] = next_state_index;
      open.push(NodeRecord{next.g + heuristic(next, goal), next_state_index});
    }
  }

  ROS_WARN("Hybrid A* failed after %d iterations with %zu states", iterations, states.size());
  return false;
}

bool HybridAStarGlobalPlanner::isGoalState(const State& state, const geometry_msgs::PoseStamped& goal) const
{
  const double dx = state.x - goal.pose.position.x;
  const double dy = state.y - goal.pose.position.y;
  if (std::hypot(dx, dy) > goal_xy_tolerance_)
  {
    return false;
  }
  return std::abs(shortestAngularDistance(state.yaw, yawFromPose(goal))) <= goal_yaw_tolerance_;
}

bool HybridAStarGlobalPlanner::isStateValid(const State& state) const
{
  double cost = 0.0;
  return footprintCost(state, cost);
}

bool HybridAStarGlobalPlanner::footprintCost(const State& state, double& cost) const
{
  cost = 0.0;

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!worldToCell(state.x, state.y, mx, my))
  {
    return false;
  }
  const unsigned char center_cost = costmap_->getCost(mx, my);
  if (center_cost == costmap_2d::NO_INFORMATION && !allow_unknown_)
  {
    return false;
  }
  if (center_cost >= lethal_cost_ && center_cost != costmap_2d::NO_INFORMATION)
  {
    return false;
  }
  cost = std::max(cost, obstacleCost(center_cost));

  const double model_cost = world_model_->footprintCost(state.x, state.y, state.yaw, footprint_);
  if (model_cost < 0.0)
  {
    return allow_unknown_ && model_cost == -2.0;
  }
  cost = std::max(cost, obstacleCost(static_cast<unsigned char>(std::min(252.0, model_cost))));

  return true;
}

bool HybridAStarGlobalPlanner::traversedMotionIsValid(const State& from, const State& to, double& cost) const
{
  cost = 0.0;
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double distance = std::hypot(dx, dy);
  const double yaw_delta = shortestAngularDistance(from.yaw, to.yaw);
  const int checks = std::max(2, static_cast<int>(std::ceil(distance / std::max(0.02, costmap_->getResolution()))));

  for (int index = 1; index <= checks; ++index)
  {
    const double ratio = static_cast<double>(index) / static_cast<double>(checks);
    State sample;
    sample.x = from.x + dx * ratio;
    sample.y = from.y + dy * ratio;
    sample.yaw = normalizeAngle(from.yaw + yaw_delta * ratio);

    double sample_cost = 0.0;
    if (!footprintCost(sample, sample_cost))
    {
      return false;
    }
    cost += (distance / static_cast<double>(checks)) * (neutral_cost_ + sample_cost);
  }

  return true;
}

bool HybridAStarGlobalPlanner::worldToCell(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  return costmap_->worldToMap(wx, wy, mx, my);
}

unsigned int HybridAStarGlobalPlanner::cellIndex(unsigned int mx, unsigned int my) const
{
  return my * costmap_->getSizeInCellsX() + mx;
}

int HybridAStarGlobalPlanner::yawIndex(double yaw) const
{
  const double normalized = normalizeAngle(yaw);
  const double shifted = normalized + kPi;
  int index = static_cast<int>(std::floor(shifted / (2.0 * kPi) * static_cast<double>(yaw_bins_)));
  if (index >= yaw_bins_)
  {
    index = yaw_bins_ - 1;
  }
  return std::max(0, index);
}

std::uint64_t HybridAStarGlobalPlanner::stateKeyIndex(const StateKey& key) const
{
  const std::uint64_t cell_count =
      static_cast<std::uint64_t>(costmap_->getSizeInCellsX()) *
      static_cast<std::uint64_t>(costmap_->getSizeInCellsY());
  return static_cast<std::uint64_t>(key.yaw) * cell_count +
         static_cast<std::uint64_t>(cellIndex(static_cast<unsigned int>(key.x),
                                             static_cast<unsigned int>(key.y)));
}

HybridAStarGlobalPlanner::StateKey HybridAStarGlobalPlanner::stateKey(const State& state) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!worldToCell(state.x, state.y, mx, my))
  {
    return StateKey{-1, -1, yawIndex(state.yaw)};
  }
  return StateKey{static_cast<int>(mx), static_cast<int>(my), yawIndex(state.yaw)};
}

double HybridAStarGlobalPlanner::heuristic(const State& state, const geometry_msgs::PoseStamped& goal) const
{
  const double dx = state.x - goal.pose.position.x;
  const double dy = state.y - goal.pose.position.y;
  const double distance = std::hypot(dx, dy);
  const double heading_error = std::abs(shortestAngularDistance(state.yaw, std::atan2(-dy, -dx)));
  return distance + minimum_turning_radius_ * std::min(heading_error, 1.0);
}

double HybridAStarGlobalPlanner::obstacleCost(unsigned char cost) const
{
  if (cost == costmap_2d::NO_INFORMATION)
  {
    return allow_unknown_ ? 0.0 : std::numeric_limits<double>::infinity();
  }
  return cost_factor_ * static_cast<double>(cost) / 252.0;
}

std::vector<HybridAStarGlobalPlanner::State> HybridAStarGlobalPlanner::successors(const State& state) const
{
  std::vector<State> result;
  result.reserve(allow_reverse_ ? 6 : 3);

  const double max_delta_yaw = step_size_ / minimum_turning_radius_;
  const std::vector<double> steering_yaw_deltas{-max_delta_yaw, 0.0, max_delta_yaw};
  const std::vector<int> directions = allow_reverse_ ? std::vector<int>{1, -1} : std::vector<int>{1};

  for (const int direction : directions)
  {
    for (const double yaw_delta_forward : steering_yaw_deltas)
    {
      const double signed_step = static_cast<double>(direction) * step_size_;
      const double yaw_delta = static_cast<double>(direction) * yaw_delta_forward;
      State next;
      next.reverse = direction < 0;

      if (std::abs(yaw_delta) < 1e-6)
      {
        next.x = state.x + signed_step * std::cos(state.yaw);
        next.y = state.y + signed_step * std::sin(state.yaw);
        next.yaw = state.yaw;
      }
      else
      {
        const double radius = signed_step / yaw_delta;
        next.x = state.x + radius * (std::sin(state.yaw + yaw_delta) - std::sin(state.yaw));
        next.y = state.y - radius * (std::cos(state.yaw + yaw_delta) - std::cos(state.yaw));
        next.yaw = normalizeAngle(state.yaw + yaw_delta);
      }

      next.g = 0.0;
      next.penalty = 0.0;
      next.parent = -1;
      if (std::abs(yaw_delta_forward) > 1e-6)
      {
        next.penalty += steering_change_penalty_;
      }
      result.push_back(next);
    }
  }

  return result;
}

void HybridAStarGlobalPlanner::buildPlan(const std::vector<State>& states,
                                         int goal_state_index,
                                         const geometry_msgs::PoseStamped& start,
                                         const geometry_msgs::PoseStamped& goal,
                                         std::vector<geometry_msgs::PoseStamped>& plan) const
{
  std::vector<State> reversed_states;
  int index = goal_state_index;
  while (index >= 0)
  {
    reversed_states.push_back(states[index]);
    index = states[index].parent;
  }
  std::reverse(reversed_states.begin(), reversed_states.end());

  plan.reserve(reversed_states.size() + 2);
  for (const State& state : reversed_states)
  {
    plan.push_back(poseFromState(state, start));
  }

  if (!plan.empty())
  {
    plan.front().header = start.header;
    plan.front().pose = start.pose;
    plan.back().header = goal.header;
    plan.back().pose.position = goal.pose.position;
    setYaw(plan.back(), yawFromPose(goal));
  }
}

geometry_msgs::PoseStamped HybridAStarGlobalPlanner::poseFromState(
    const State& state,
    const geometry_msgs::PoseStamped& reference) const
{
  geometry_msgs::PoseStamped pose;
  pose.header = reference.header;
  pose.header.frame_id = global_frame_;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = state.x;
  pose.pose.position.y = state.y;
  pose.pose.position.z = 0.0;
  setYaw(pose, state.yaw);
  return pose;
}

void HybridAStarGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) const
{
  if (!publish_plan_ || !plan_pub_)
  {
    return;
  }

  nav_msgs::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = ros::Time::now();
  path.poses = plan;
  for (geometry_msgs::PoseStamped& pose : path.poses)
  {
    pose.header = path.header;
  }
  plan_pub_.publish(path);
}

double HybridAStarGlobalPlanner::normalizeAngle(double angle)
{
  while (angle > kPi)
  {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi)
  {
    angle += 2.0 * kPi;
  }
  return angle;
}

double HybridAStarGlobalPlanner::shortestAngularDistance(double from, double to)
{
  return normalizeAngle(to - from);
}

double HybridAStarGlobalPlanner::yawFromPose(const geometry_msgs::PoseStamped& pose)
{
  tf2::Quaternion quaternion;
  tf2::fromMsg(pose.pose.orientation, quaternion);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

void HybridAStarGlobalPlanner::setYaw(geometry_msgs::PoseStamped& pose, double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(quaternion);
}

}  // namespace zebrat

PLUGINLIB_EXPORT_CLASS(zebrat::HybridAStarGlobalPlanner, nav_core::BaseGlobalPlanner)
