#ifndef ZEBRAT_ACKERMANN_PURE_PURSUIT_LOCAL_PLANNER_H
#define ZEBRAT_ACKERMANN_PURE_PURSUIT_LOCAL_PLANNER_H

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>
#include <vector>

namespace zebrat
{

class AckermannPurePursuitLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
  AckermannPurePursuitLocalPlanner();

  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

private:
  bool transformPlanToBase(std::vector<geometry_msgs::PoseStamped>& transformed_plan) const;
  bool selectCarrot(const std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                    geometry_msgs::PoseStamped& carrot) const;
  bool getGoalInFrame(const std::string& target_frame, geometry_msgs::PoseStamped& goal) const;
  bool commandIsCollisionFree(double speed, double steering_angle) const;
  bool poseIsCollisionFree(double x, double y, double yaw) const;
  double goalDistanceInBase() const;
  double velocityForGoalDistance(double goal_distance, double steering_angle, bool reverse) const;
  void publishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& transformed_plan) const;
  static double clamp(double value, double lower, double upper);
  static double normalizeAngle(double angle);
  static double yawFromPose(const geometry_msgs::PoseStamped& pose);

  bool initialized_ = false;
  tf2_ros::Buffer* tf_ = nullptr;
  costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
  costmap_2d::Costmap2D* costmap_ = nullptr;
  std::unique_ptr<base_local_planner::CostmapModel> world_model_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::Point> footprint_;
  ros::Publisher local_plan_pub_;

  std::string global_frame_;
  std::string base_frame_;
  double transform_tolerance_ = 0.10;
  double wheelbase_ = 0.41893;
  double lookahead_distance_ = 0.80;
  double min_lookahead_distance_ = 0.35;
  double max_vel_x_ = 0.22;
  double min_vel_x_ = 0.04;
  double max_vel_x_backwards_ = 0.06;
  double max_steering_angle_ = 0.42;
  double slowdown_distance_ = 0.80;
  double xy_goal_tolerance_ = 0.30;
  double yaw_goal_tolerance_ = 0.35;
  double prune_distance_ = 0.25;
  double steering_speed_scale_ = 0.55;
  double collision_check_time_ = 1.20;
  double collision_check_step_ = 0.10;
  bool allow_reverse_ = true;
  bool publish_local_plan_ = true;
};

}  // namespace zebrat

#endif  // ZEBRAT_ACKERMANN_PURE_PURSUIT_LOCAL_PLANNER_H
