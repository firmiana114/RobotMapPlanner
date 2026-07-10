#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace rmp {

constexpr std::uint8_t kFree = 0;
constexpr std::uint8_t kInscribed = 253;
constexpr std::uint8_t kOccupied = 254;
constexpr std::uint8_t kUnknown = 255;
constexpr std::uint8_t kOverlayInherit = 0;
constexpr std::uint8_t kOverlayFree = 1;
constexpr std::uint8_t kOverlayOccupied = 2;

struct Point3 {
  double x{};
  double y{};
  double z{};
};

struct MapConfig {
  double resolution{0.10};
  double obstacle_min_height{0.15};
  double obstacle_max_height{2.00};
  int min_points_per_cell{1};
};

struct GridMeta {
  int width{};
  int height{};
  double origin_x{};
  double origin_y{};
  double resolution{0.10};
  double ground_z{};
};

struct BuildResult {
  GridMeta meta;
  std::vector<std::uint8_t> obstacles;
  std::vector<std::uint8_t> base_grid;
  std::vector<std::pair<double, double>> boundary;
  std::uint64_t declared_points{};
  std::uint64_t finite_points{};
  std::uint64_t obstacle_points{};
  std::uint64_t occupied_cells{};
  std::string data_encoding;
  Point3 min_bound;
  Point3 max_bound;
};

struct CostConfig {
  double hard_clearance{0.25};
  double inflation_radius{0.50};
  double cost_scaling{5.0};
};

struct PlanConfig {
  double snap_radius{0.50};
  double point_spacing{0.50};
  double cost_weight{2.0};
};

struct PlanResult {
  bool ok{false};
  std::string error_code;
  std::string message;
  std::pair<double, double> requested_start;
  std::pair<double, double> requested_goal;
  std::pair<double, double> actual_start;
  std::pair<double, double> actual_goal;
  bool start_snapped{false};
  bool goal_snapped{false};
  std::vector<std::pair<double, double>> points;
  double length_m{};
  double total_cost{};
  std::uint64_t expanded_nodes{};
};

struct ValidationResult {
  std::uint64_t free_cells{};
  std::uint64_t occupied_cells{};
  std::uint64_t unknown_cells{};
  std::uint64_t traversable_cells{};
  std::uint64_t connected_components{};
};

BuildResult BuildMapFromPcd(const std::string& path, const MapConfig& config);
std::vector<std::uint8_t> ApplyBoundary(
    const std::vector<std::uint8_t>& obstacles, const GridMeta& meta,
    const std::vector<std::pair<double, double>>& boundary);
std::vector<std::uint8_t> MergeOverlay(
    const std::vector<std::uint8_t>& base,
    const std::vector<std::uint8_t>& overlay);
std::vector<std::uint8_t> BuildCostmap(
    const std::vector<std::uint8_t>& final_grid, const GridMeta& meta,
    const CostConfig& config);
ValidationResult ValidateGrid(
    const std::vector<std::uint8_t>& final_grid,
    const std::vector<std::uint8_t>& costmap, const GridMeta& meta);
PlanResult PlanPath(
    const std::vector<std::uint8_t>& costmap, const GridMeta& meta,
    std::pair<double, double> start, std::pair<double, double> goal,
    const PlanConfig& config);

}  // namespace rmp
