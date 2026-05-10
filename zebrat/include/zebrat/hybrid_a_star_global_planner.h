#ifndef ZEBRAT_HYBRID_A_STAR_GLOBAL_PLANNER_H
#define ZEBRAT_HYBRID_A_STAR_GLOBAL_PLANNER_H

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace zebrat
{

class HybridAStarGlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
  HybridAStarGlobalPlanner();
  HybridAStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan,
                double& cost) override;

private:
  struct State
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double g = 0.0;
    double penalty = 0.0;
    int parent = -1;
    bool reverse = false;
  };

  struct NodeRecord
  {
    double f = 0.0;
    int state_index = -1;
  };

  struct NodeRecordCompare
  {
    bool operator()(const NodeRecord& left, const NodeRecord& right) const
    {
      return left.f > right.f;
    }
  };

  struct StateKey
  {
    int x = 0;
    int y = 0;
    int yaw = 0;
  };

  bool planHybridAStar(const geometry_msgs::PoseStamped& start,
                       const geometry_msgs::PoseStamped& goal,
                       std::vector<State>& states,
                       int& goal_state_index,
                       double& path_cost);
  bool isGoalState(const State& state, const geometry_msgs::PoseStamped& goal) const;
  bool isStateValid(const State& state) const;
  bool footprintCost(const State& state, double& cost) const;
  bool traversedMotionIsValid(const State& from, const State& to, double& cost) const;
  bool worldToCell(double wx, double wy, unsigned int& mx, unsigned int& my) const;
  unsigned int cellIndex(unsigned int mx, unsigned int my) const;
  int yawIndex(double yaw) const;
  std::uint64_t stateKeyIndex(const StateKey& key) const;
  StateKey stateKey(const State& state) const;
  double heuristic(const State& state, const geometry_msgs::PoseStamped& goal) const;
  double obstacleCost(unsigned char cost) const;
  std::vector<State> successors(const State& state) const;
  void buildPlan(const std::vector<State>& states,
                 int goal_state_index,
                 const geometry_msgs::PoseStamped& start,
                 const geometry_msgs::PoseStamped& goal,
                 std::vector<geometry_msgs::PoseStamped>& plan) const;
  geometry_msgs::PoseStamped poseFromState(const State& state,
                                           const geometry_msgs::PoseStamped& reference) const;
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) const;
  static double normalizeAngle(double angle);
  static double shortestAngularDistance(double from, double to);
  static double yawFromPose(const geometry_msgs::PoseStamped& pose);
  static void setYaw(geometry_msgs::PoseStamped& pose, double yaw);

  bool initialized_ = false;
  costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
  costmap_2d::Costmap2D* costmap_ = nullptr;
  std::unique_ptr<base_local_planner::CostmapModel> world_model_;
  std::string global_frame_;
  ros::Publisher plan_pub_;

  double minimum_turning_radius_ = 0.98;
  double step_size_ = 0.20;
  int yaw_bins_ = 72;
  int max_iterations_ = 30000;
  bool allow_reverse_ = true;
  bool allow_unknown_ = true;
  unsigned char lethal_cost_ = 253;
  double neutral_cost_ = 1.0;
  double cost_factor_ = 2.0;
  double reverse_penalty_ = 1.8;
  double direction_change_penalty_ = 1.5;
  double steering_change_penalty_ = 0.15;
  double goal_xy_tolerance_ = 0.25;
  double goal_yaw_tolerance_ = 0.35;
  double analytic_expansion_max_length_ = 0.60;
  double footprint_padding_ = 0.01;
  bool publish_plan_ = true;

  std::vector<geometry_msgs::Point> footprint_;
};

}  // namespace zebrat

#endif  // ZEBRAT_HYBRID_A_STAR_GLOBAL_PLANNER_H
