#include "rmp/core.hpp"

#include <algorithm>
#include <cassert>
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
  const auto validation = rmp::ValidateGrid(grid, costmap, meta);
  assert(validation.connected_components == 1);
  rmp::PlanConfig plan_config{0.0, 0.2, 1.0};
  const auto plan = rmp::PlanPath(costmap, meta, {0.25, 1.05}, {1.75, 1.05}, plan_config);
  assert(plan.ok);
  assert(plan.points.size() >= 2);
  assert(plan.length_m > 1.5);

  std::filesystem::remove(path);
  std::cout << "rmp_core_tests passed\n";
  return 0;
}
