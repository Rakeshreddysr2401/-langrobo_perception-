// footprint_clear — the cells under the rover's own outline are FREE.
//
// NAV_PLAN.md N6. The rover is physically standing there, so any mark in them
// is stale by definition: nvblox remembers what the camera saw on the way in
// (it cannot see within ~30 cm of the nose), and its layer has no footprint
// clearing of its own. Drive test 2026-09-27: the lattice planner refused the
// rover's own pose as a start ("Start occupied") six times, with the raw
// LiDAR + depth showing 2-4 cm of real clearance.
//
// THE HEADING IS BINNED. The lattice checks its start pose at the nearest of
// its 16 headings (22.5 deg apart), so the outline it tests can be turned up
// to 11.25 deg from where the rover faces -- ~5 cm at the corners. Clearing
// the outline at the true heading alone left "Start occupied" in a pocket
// with 0.7 cm of real room (drive test 2026-09-28). So the outline is cleared
// across heading_sweep either side, in steps of <= 2.5 deg.
//
// GLOBAL costmap only, after nvblox + LiDAR and before inflation. The local
// costmap that MPPI checks every trajectory against keeps every mark, so the
// collision check that protects the body is unchanged: this only stops the
// PLANNER from rejecting where the rover already is.
#include <algorithm>
#include <cmath>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rover_costmap_plugins
{

class FootprintClearLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    declareParameter("enabled", rclcpp::ParameterValue(true));
    // beyond the costmap's padded footprint: the start check rasterises the
    // outline's EDGES, so a cell whose centre is up to half a cell outside
    // still counts -- clear that half cell too
    declareParameter("extra_padding", rclcpp::ParameterValue(0.02));
    // half the lattice's heading bin: 2*pi/16/2
    declareParameter("heading_sweep", rclcpp::ParameterValue(0.19635));
    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".extra_padding", extra_);
    node->get_parameter(name_ + ".heading_sweep", sweep_);
    current_ = true;
  }

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    if (!enabled_) {
      return;
    }
    std::vector<geometry_msgs::msg::Point> fp = layered_costmap_->getFootprint();
    if (extra_ > 0.0) {
      nav2_costmap_2d::padFootprint(fp, extra_);
    }
    outlines_.clear();
    const int n = std::max(1, static_cast<int>(std::ceil(sweep_ / 0.0436)));   // 2.5 deg steps
    for (int k = -n; k <= n; ++k) {
      std::vector<geometry_msgs::msg::Point> o;
      nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw + sweep_ * k / n, fp, o);
      for (const auto & p : o) {
        *min_x = std::min(*min_x, p.x);
        *min_y = std::min(*min_y, p.y);
        *max_x = std::max(*max_x, p.x);
        *max_y = std::max(*max_y, p.y);
      }
      outlines_.push_back(o);
    }
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master, int, int, int, int) override
  {
    if (!enabled_) {
      return;
    }
    for (const auto & outline : outlines_) {
      std::vector<nav2_costmap_2d::MapLocation> polygon, cells;
      bool inside = true;
      for (const auto & p : outline) {
        unsigned int mx, my;
        if (!master.worldToMap(p.x, p.y, mx, my)) {
          inside = false;             // this outline leaves the map: skip it
          break;
        }
        polygon.push_back({mx, my});
      }
      if (!inside) {
        continue;
      }
      master.convexFillCells(polygon, cells);
      for (const auto & c : cells) {
        master.setCost(c.x, c.y, nav2_costmap_2d::FREE_SPACE);
      }
    }
  }

  void reset() override {}
  bool isClearable() override {return false;}

private:
  bool enabled_{true};
  double extra_{0.02};
  double sweep_{0.19635};
  std::vector<std::vector<geometry_msgs::msg::Point>> outlines_;
};

}  // namespace rover_costmap_plugins

PLUGINLIB_EXPORT_CLASS(rover_costmap_plugins::FootprintClearLayer, nav2_costmap_2d::Layer)
