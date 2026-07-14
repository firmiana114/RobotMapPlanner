#include "rmp/core.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
  const auto path = std::filesystem::temp_directory_path() / "rmp_core_test.pcd";
  {
    std::ofstream output(path);
    output << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
              "COUNT 1 1 1\nWIDTH 12\nHEIGHT 1\nPOINTS 12\nDATA ascii\n";
    output << "0 0 0\n0 0 0.5\n0 2 0\n0 2 0.5\n2 0 0\n2 0 0.5\n2 2 0\n2 2 0.5\n"
              "1 0 0\n1 2 0\n0 1 0\n2 1 0\n";
  }
  rmp::MapConfig build_config;
  build_config.resolution = 0.25;
  build_config.obstacle_min_height = 0.20;
  build_config.obstacle_max_height = 1.0;
  const auto built = rmp::BuildMapFromPcd(path.string(), build_config);
  assert(built.declared_points == 12);
  assert(built.finite_points == 12);
  assert(built.occupied_cells == 4);
  assert(built.boundary.size() == 4);
  assert(built.base_grid.size() == static_cast<std::size_t>(built.meta.width * built.meta.height));

  std::vector<std::uint8_t> overlay(built.base_grid.size(), rmp::kOverlayInherit);
  auto first_obstacle = std::find(built.base_grid.begin(), built.base_grid.end(), rmp::kOccupied);
  assert(first_obstacle != built.base_grid.end());
  overlay[static_cast<std::size_t>(first_obstacle - built.base_grid.begin())] = rmp::kOverlayFree;
  const auto merged = rmp::MergeOverlay(built.base_grid, overlay);
  assert(merged[static_cast<std::size_t>(first_obstacle - built.base_grid.begin())] == rmp::kFree);

  rmp::GridMeta meta{20, 20, 0.0, 0.0, 0.1, 0.0};
  std::vector<std::uint8_t> grid(400, rmp::kFree);
  for (int y = 4; y < 16; ++y) grid[static_cast<std::size_t>(y * 20 + 10)] = rmp::kOccupied;
  rmp::CostConfig cost_config{0.0, 0.15, 3.0};
  const auto costmap = rmp::BuildCostmap(grid, meta, cost_config);
  assert(costmap[10 * 20 + 10] == rmp::kOccupied);
  auto invalid_cost_config = cost_config;
  invalid_cost_config.inflation_radius = -0.1;
  try {
    static_cast<void>(rmp::BuildCostmap(grid, meta, invalid_cost_config));
    assert(false && "invalid costmap parameters must be rejected");
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    assert(message.find("inflation_radius >= hard_clearance") != std::string::npos);
    assert(message.find("0, -0.1, 3") != std::string::npos);
  }
  const auto validation = rmp::ValidateGrid(grid, costmap, meta);
  assert(validation.connected_components == 1);
  rmp::PlanConfig plan_config{0.0, 0.2, 1.0};
  const auto plan = rmp::PlanPath(costmap, meta, {0.25, 1.05}, {1.75, 1.05}, plan_config);
  assert(plan.ok);
  assert(plan.points.size() >= 2);
  assert(plan.length_m > 1.5);

  rmp::GridMeta threshold_meta{7, 5, 0.0, 0.0, 0.1, 0.0};
  std::vector<std::uint8_t> threshold_costmap(35, rmp::kFree);
  for (int y = 0; y < threshold_meta.height; ++y) {
    threshold_costmap[static_cast<std::size_t>(y * threshold_meta.width + 3)] = 10;
  }
  rmp::PlanConfig strict_config{0.0, 0.05, 1.0, 0};
  const auto strict_plan = rmp::PlanPath(
      threshold_costmap, threshold_meta, {0.15, 0.25}, {0.55, 0.25}, strict_config);
  assert(!strict_plan.ok);
  assert(strict_plan.error_code == "NO_PATH");
  auto permissive_config = strict_config;
  permissive_config.max_traversable_cost = 10;
  const auto permissive_plan = rmp::PlanPath(
      threshold_costmap, threshold_meta, {0.15, 0.25}, {0.55, 0.25}, permissive_config);
  assert(permissive_plan.ok);

  rmp::GridMeta corner_meta{5, 5, 0.0, 0.0, 0.1, 0.0};
  std::vector<std::uint8_t> corner_costmap(25, rmp::kFree);
  corner_costmap[1] = rmp::kInscribed;
  const auto corner_plan = rmp::PlanPath(
      corner_costmap, corner_meta, {0.05, 0.05}, {0.45, 0.45}, strict_config);
  assert(corner_plan.ok);
  assert(corner_plan.length_m > std::hypot(0.4, 0.4) + 0.02);
  for (std::size_t i = 1; i < corner_plan.points.size(); ++i) {
    const auto a = corner_plan.points[i - 1];
    const auto b = corner_plan.points[i];
    const auto distance = std::hypot(b.first - a.first, b.second - a.second);
    assert(distance <= strict_config.point_spacing + 1e-9);
    const int samples = std::max(1, static_cast<int>(std::ceil(distance / 0.005)));
    for (int sample = 0; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      const int x = static_cast<int>(std::floor((a.first + ratio * (b.first - a.first)) / 0.1));
      const int y = static_cast<int>(std::floor((a.second + ratio * (b.second - a.second)) / 0.1));
      assert(corner_costmap[static_cast<std::size_t>(y * corner_meta.width + x)] == rmp::kFree);
    }
  }

  std::filesystem::remove(path);
  std::cout << "rmp_core_tests passed\n";
  return 0;
}
