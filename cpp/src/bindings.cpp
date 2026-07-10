#include "rmp/core.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace {
std::vector<std::uint8_t> FromBytes(const py::bytes& value) {
  std::string data = value;
  return {data.begin(), data.end()};
}
py::bytes ToBytes(const std::vector<std::uint8_t>& value) {
  return py::bytes(reinterpret_cast<const char*>(value.data()), value.size());
}
rmp::GridMeta Meta(const py::dict& data) {
  return {data["width"].cast<int>(), data["height"].cast<int>(),
          data["origin_x"].cast<double>(), data["origin_y"].cast<double>(),
          data["resolution"].cast<double>(), data["ground_z"].cast<double>()};
}
py::dict MetaDict(const rmp::GridMeta& meta) {
  py::dict result;
  result["width"] = meta.width; result["height"] = meta.height;
  result["origin_x"] = meta.origin_x; result["origin_y"] = meta.origin_y;
  result["resolution"] = meta.resolution; result["ground_z"] = meta.ground_z;
  return result;
}
}  // namespace

PYBIND11_MODULE(_core, module) {
  module.doc() = "RobotMapPlanner C++ core";
  module.def("build_map", [](const std::string& path, const py::dict& config) {
    rmp::MapConfig cfg;
    if (config.contains("resolution")) cfg.resolution = config["resolution"].cast<double>();
    if (config.contains("obstacle_min_height")) cfg.obstacle_min_height = config["obstacle_min_height"].cast<double>();
    if (config.contains("obstacle_max_height")) cfg.obstacle_max_height = config["obstacle_max_height"].cast<double>();
    if (config.contains("min_points_per_cell")) cfg.min_points_per_cell = config["min_points_per_cell"].cast<int>();
    const auto value = rmp::BuildMapFromPcd(path, cfg);
    py::dict result;
    result["meta"] = MetaDict(value.meta); result["obstacles"] = ToBytes(value.obstacles);
    result["base_grid"] = ToBytes(value.base_grid); result["boundary"] = value.boundary;
    result["declared_points"] = value.declared_points; result["finite_points"] = value.finite_points;
    result["obstacle_points"] = value.obstacle_points; result["occupied_cells"] = value.occupied_cells;
    result["data_encoding"] = value.data_encoding;
    result["min_bound"] = py::make_tuple(value.min_bound.x, value.min_bound.y, value.min_bound.z);
    result["max_bound"] = py::make_tuple(value.max_bound.x, value.max_bound.y, value.max_bound.z);
    return result;
  });
  module.def("apply_boundary", [](const py::bytes& obstacles, const py::dict& meta,
                                    const std::vector<std::pair<double, double>>& boundary) {
    return ToBytes(rmp::ApplyBoundary(FromBytes(obstacles), Meta(meta), boundary));
  });
  module.def("merge_overlay", [](const py::bytes& base, const py::bytes& overlay) {
    return ToBytes(rmp::MergeOverlay(FromBytes(base), FromBytes(overlay)));
  });
  module.def("build_costmap", [](const py::bytes& grid, const py::dict& meta, const py::dict& config) {
    rmp::CostConfig cfg;
    if (config.contains("hard_clearance")) cfg.hard_clearance = config["hard_clearance"].cast<double>();
    if (config.contains("inflation_radius")) cfg.inflation_radius = config["inflation_radius"].cast<double>();
    if (config.contains("cost_scaling")) cfg.cost_scaling = config["cost_scaling"].cast<double>();
    return ToBytes(rmp::BuildCostmap(FromBytes(grid), Meta(meta), cfg));
  });
  module.def("validate_grid", [](const py::bytes& grid, const py::bytes& costmap, const py::dict& meta) {
    const auto value = rmp::ValidateGrid(FromBytes(grid), FromBytes(costmap), Meta(meta));
    py::dict result; result["free_cells"] = value.free_cells; result["occupied_cells"] = value.occupied_cells;
    result["unknown_cells"] = value.unknown_cells; result["traversable_cells"] = value.traversable_cells;
    result["connected_components"] = value.connected_components; return result;
  });
  module.def("plan", [](const py::bytes& costmap, const py::dict& meta,
                         std::pair<double, double> start, std::pair<double, double> goal,
                         const py::dict& config) {
    rmp::PlanConfig cfg;
    if (config.contains("snap_radius")) cfg.snap_radius = config["snap_radius"].cast<double>();
    if (config.contains("point_spacing")) cfg.point_spacing = config["point_spacing"].cast<double>();
    if (config.contains("cost_weight")) cfg.cost_weight = config["cost_weight"].cast<double>();
    const auto value = rmp::PlanPath(FromBytes(costmap), Meta(meta), start, goal, cfg);
    py::dict result; result["ok"] = value.ok; result["error_code"] = value.error_code;
    result["message"] = value.message; result["requested_start"] = value.requested_start;
    result["requested_goal"] = value.requested_goal; result["actual_start"] = value.actual_start;
    result["actual_goal"] = value.actual_goal; result["start_snapped"] = value.start_snapped;
    result["goal_snapped"] = value.goal_snapped; result["points"] = value.points;
    result["length_m"] = value.length_m; result["total_cost"] = value.total_cost;
    result["expanded_nodes"] = value.expanded_nodes; return result;
  });
}
