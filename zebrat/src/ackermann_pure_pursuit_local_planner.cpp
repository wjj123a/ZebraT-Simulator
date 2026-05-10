#include <zebrat/ackermann_pure_pursuit_local_planner.h>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace zebrat
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

AckermannPurePursuitLocalPlanner::AckermannPurePursuitLocalPlanner() = default;

void AckermannPurePursuitLocalPlanner::initialize(std::string name,
                                                  tf2_ros::Buffer* tf,
                                                  costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("AckermannPurePursuitLocalPlanner has already been initialized");
    return;
  }

  if (tf == nullptr || costmap_ros == nullptr)
  {
    ROS_ERROR("AckermannPurePursuitLocalPlanner requires tf and costmap");
    return;
  }

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  world_model_.reset(new base_local_planner::CostmapModel(*costmap_));
  global_frame_ = costmap_ros_->getGlobalFrameID();
  base_frame_ = costmap_ros_->getBaseFrameID();
  footprint_ = costmap_ros_->getRobotFootprint();

  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("transform_tolerance", transform_tolerance_, transform_tolerance_);
  private_nh.param("wheelbase", wheelbase_, wheelbase_);
  private_nh.param("lookahead_distance", lookahead_distance_, lookahead_distance_);
  private_nh.param("min_lookahead_distance", min_lookahead_distance_, min_lookahead_distance_);
  private_nh.param("max_vel_x", max_vel_x_, max_vel_x_);
  private_nh.param("min_vel_x", min_vel_x_, min_vel_x_);
  private_nh.param("max_vel_x_backwards", max_vel_x_backwards_, max_vel_x_backwards_);
  private_nh.param("max_steering_angle", max_steering_angle_, max_steering_angle_);
  private_nh.param("slowdown_distance", slowdown_distance_, slowdown_distance_);
  private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, xy_goal_tolerance_);
  private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, yaw_goal_tolerance_);
  private_nh.param("prune_distance", prune_distance_, prune_distance_);
  private_nh.param("steering_speed_scale", steering_speed_scale_, steering_speed_scale_);
  private_nh.param("collision_check_time", collision_check_time_, collision_check_time_);
  private_nh.param("collision_check_step", collision_check_step_, collision_check_step_);
  private_nh.param("allow_reverse", allow_reverse_, allow_reverse_);
  private_nh.param("publish_local_plan", publish_local_plan_, publish_local_plan_);

  wheelbase_ = std::max(0.05, wheelbase_);
  lookahead_distance_ = std::max(0.05, lookahead_distance_);
  min_lookahead_distance_ = std::max(0.02, std::min(min_lookahead_distance_, lookahead_distance_));
  max_vel_x_ = std::max(0.0, max_vel_x_);
  min_vel_x_ = clamp(min_vel_x_, 0.0, max_vel_x_);
  max_vel_x_backwards_ = std::max(0.0, max_vel_x_backwards_);
  max_steering_angle_ = std::max(0.01, max_steering_angle_);
  slowdown_distance_ = std::max(xy_goal_tolerance_, slowdown_distance_);
  xy_goal_tolerance_ = std::max(0.01, xy_goal_tolerance_);
  yaw_goal_tolerance_ = std::max(0.01, yaw_goal_tolerance_);
  prune_distance_ = std::max(0.0, prune_distance_);
  steering_speed_scale_ = clamp(steering_speed_scale_, 0.0, 1.0);
  collision_check_time_ = std::max(0.0, collision_check_time_);
  collision_check_step_ = std::max(0.02, collision_check_step_);

  local_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1, true);
  initialized_ = true;
  ROS_INFO("Initialized Ackermann Pure Pursuit local planner with %.2fm lookahead", lookahead_distance_);
}

bool AckermannPurePursuitLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("AckermannPurePursuitLocalPlanner has not been initialized");
    return false;
  }
  global_plan_ = plan;
  return !global_plan_.empty();
}

bool AckermannPurePursuitLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  cmd_vel = geometry_msgs::Twist();
  if (!initialized_ || global_plan_.empty())
  {
    return false;
  }

  if (isGoalReached())
  {
    return true;
  }

  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if (!transformPlanToBase(transformed_plan) || transformed_plan.empty())
  {
    ROS_WARN_THROTTLE(1.0, "Pure Pursuit could not transform the global plan");
    return false;
  }
  publishLocalPlan(transformed_plan);

  geometry_msgs::PoseStamped carrot;
  if (!selectCarrot(transformed_plan, carrot))
  {
    ROS_WARN_THROTTLE(1.0, "Pure Pursuit could not select a lookahead point");
    return false;
  }

  const double x = carrot.pose.position.x;
  const double y = carrot.pose.position.y;
  const double distance2 = x * x + y * y;
  if (distance2 < 1e-6)
  {
    return true;
  }

  const bool reverse = allow_reverse_ && x < -min_lookahead_distance_;
  const double curvature = 2.0 * y / distance2;
  double steering_angle = std::atan(wheelbase_ * curvature);
  if (reverse)
  {
    steering_angle = -steering_angle;
  }
  steering_angle = clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

  const double goal_distance = goalDistanceInBase();
  double speed = velocityForGoalDistance(goal_distance, steering_angle, reverse);
  if (reverse)
  {
    speed = -speed;
  }

  if (!commandIsCollisionFree(speed, steering_angle))
  {
    ROS_WARN_THROTTLE(1.0, "Pure Pursuit command blocked by local costmap collision check");
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  cmd_vel.linear.x = speed;
  cmd_vel.angular.z = steering_angle;
  return true;
}

bool AckermannPurePursuitLocalPlanner::isGoalReached()
{
  if (!initialized_ || global_plan_.empty())
  {
    return false;
  }

  geometry_msgs::PoseStamped goal;
  if (!getGoalInFrame(base_frame_, goal))
  {
    return false;
  }

  const double xy_distance = std::hypot(goal.pose.position.x, goal.pose.position.y);
  const double yaw_error = std::abs(normalizeAngle(yawFromPose(goal)));
  return xy_distance <= xy_goal_tolerance_ && yaw_error <= yaw_goal_tolerance_;
}

bool AckermannPurePursuitLocalPlanner::transformPlanToBase(
    std::vector<geometry_msgs::PoseStamped>& transformed_plan) const
{
  transformed_plan.clear();
  std::vector<geometry_msgs::PoseStamped> all_transformed;
  all_transformed.reserve(global_plan_.size());

  for (const geometry_msgs::PoseStamped& pose : global_plan_)
  {
    if (pose.header.frame_id.empty())
    {
      continue;
    }

    geometry_msgs::PoseStamped transformed;
    try
    {
      tf_->transform(pose, transformed, base_frame_, ros::Duration(transform_tolerance_));
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1.0, "Pure Pursuit transform failed: %s", ex.what());
      return false;
    }

    all_transformed.push_back(transformed);
  }

  if (all_transformed.empty())
  {
    return false;
  }

  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < all_transformed.size(); ++index)
  {
    const double x = all_transformed[index].pose.position.x;
    const double y = all_transformed[index].pose.position.y;
    const double distance = std::hypot(x, y);
    if (distance < closest_distance)
    {
      closest_distance = distance;
      closest_index = index;
    }
  }

  if (closest_distance <= prune_distance_ && closest_index + 1 < all_transformed.size())
  {
    transformed_plan.assign(all_transformed.begin() + closest_index + 1, all_transformed.end());
  }
  else
  {
    transformed_plan.assign(all_transformed.begin() + closest_index, all_transformed.end());
  }
  return !transformed_plan.empty();
}

bool AckermannPurePursuitLocalPlanner::selectCarrot(
    const std::vector<geometry_msgs::PoseStamped>& transformed_plan,
    geometry_msgs::PoseStamped& carrot) const
{
  const double lookahead = lookahead_distance_;
  bool selected = false;

  for (const geometry_msgs::PoseStamped& pose : transformed_plan)
  {
    const double x = pose.pose.position.x;
    const double y = pose.pose.position.y;
    const double distance = std::hypot(x, y);
    if (distance < min_lookahead_distance_)
    {
      continue;
    }
    carrot = pose;
    selected = true;
    if (distance >= lookahead)
    {
      return true;
    }
  }

  return selected;
}

bool AckermannPurePursuitLocalPlanner::getGoalInFrame(const std::string& target_frame,
                                                      geometry_msgs::PoseStamped& goal) const
{
  if (global_plan_.empty())
  {
    return false;
  }

  try
  {
    tf_->transform(global_plan_.back(), goal, target_frame, ros::Duration(transform_tolerance_));
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "Pure Pursuit goal transform failed: %s", ex.what());
    return false;
  }
  return true;
}

bool AckermannPurePursuitLocalPlanner::commandIsCollisionFree(double speed, double steering_angle) const
{
  if (collision_check_time_ <= 0.0 || std::abs(speed) < 1e-4)
  {
    return true;
  }

  const double direction = speed >= 0.0 ? 1.0 : -1.0;
  const double total_distance = std::abs(speed) * collision_check_time_;
  const int steps = std::max(1, static_cast<int>(std::ceil(total_distance / collision_check_step_)));
  const double ds = direction * total_distance / static_cast<double>(steps);

  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  const double curvature = std::tan(steering_angle) / wheelbase_;

  for (int index = 0; index < steps; ++index)
  {
    if (std::abs(curvature) < 1e-6)
    {
      x += ds * std::cos(yaw);
      y += ds * std::sin(yaw);
    }
    else
    {
      const double yaw_delta = ds * curvature;
      const double radius = ds / yaw_delta;
      x += radius * (std::sin(yaw + yaw_delta) - std::sin(yaw));
      y -= radius * (std::cos(yaw + yaw_delta) - std::cos(yaw));
      yaw = normalizeAngle(yaw + yaw_delta);
    }

    if (!poseIsCollisionFree(x, y, yaw))
    {
      return false;
    }
  }

  return true;
}

bool AckermannPurePursuitLocalPlanner::poseIsCollisionFree(double x, double y, double yaw) const
{
  geometry_msgs::PoseStamped base_pose;
  base_pose.header.frame_id = base_frame_;
  base_pose.header.stamp = ros::Time(0);
  base_pose.pose.position.x = x;
  base_pose.pose.position.y = y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  base_pose.pose.orientation = tf2::toMsg(quaternion);

  geometry_msgs::PoseStamped costmap_pose;
  try
  {
    tf_->transform(base_pose, costmap_pose, global_frame_, ros::Duration(transform_tolerance_));
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "Pure Pursuit collision transform failed: %s", ex.what());
    return false;
  }

  const double cost = world_model_->footprintCost(
      costmap_pose.pose.position.x,
      costmap_pose.pose.position.y,
      yawFromPose(costmap_pose),
      footprint_);
  return cost >= 0.0;
}

double AckermannPurePursuitLocalPlanner::goalDistanceInBase() const
{
  geometry_msgs::PoseStamped goal;
  if (!getGoalInFrame(base_frame_, goal))
  {
    return std::numeric_limits<double>::infinity();
  }
  return std::hypot(goal.pose.position.x, goal.pose.position.y);
}

double AckermannPurePursuitLocalPlanner::velocityForGoalDistance(double goal_distance,
                                                                 double steering_angle,
                                                                 bool reverse) const
{
  const double max_speed = reverse ? max_vel_x_backwards_ : max_vel_x_;
  if (max_speed <= 0.0)
  {
    return 0.0;
  }

  double speed = max_speed;
  if (std::isfinite(goal_distance) && goal_distance < slowdown_distance_)
  {
    const double scale = clamp(goal_distance / slowdown_distance_, 0.0, 1.0);
    speed = std::max(min_vel_x_, max_speed * scale);
  }

  const double steering_scale = 1.0 - steering_speed_scale_ *
      std::abs(steering_angle) / max_steering_angle_;
  return clamp(speed * clamp(steering_scale, 0.25, 1.0), 0.0, max_speed);
}

void AckermannPurePursuitLocalPlanner::publishLocalPlan(
    const std::vector<geometry_msgs::PoseStamped>& transformed_plan) const
{
  if (!publish_local_plan_ || !local_plan_pub_)
  {
    return;
  }

  nav_msgs::Path path;
  path.header.frame_id = base_frame_;
  path.header.stamp = ros::Time::now();
  path.poses = transformed_plan;
  for (geometry_msgs::PoseStamped& pose : path.poses)
  {
    pose.header = path.header;
  }
  local_plan_pub_.publish(path);
}

double AckermannPurePursuitLocalPlanner::clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double AckermannPurePursuitLocalPlanner::normalizeAngle(double angle)
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

double AckermannPurePursuitLocalPlanner::yawFromPose(const geometry_msgs::PoseStamped& pose)
{
  tf2::Quaternion quaternion;
  tf2::fromMsg(pose.pose.orientation, quaternion);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace zebrat

PLUGINLIB_EXPORT_CLASS(zebrat::AckermannPurePursuitLocalPlanner, nav_core::BaseLocalPlanner)
