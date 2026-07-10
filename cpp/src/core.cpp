#include "rmp/core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace rmp {
namespace {

struct PcdHeader {
  std::vector<std::string> fields;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<int> counts;
  std::uint64_t points{};
  std::string data;
};

std::vector<std::string> Split(const std::string& line) {
  std::istringstream input(line);
  std::vector<std::string> parts;
  std::string part;
  while (input >> part) parts.push_back(part);
  return parts;
}

PcdHeader ReadHeader(std::ifstream& input) {
  PcdHeader header;
  std::string line;
  std::uint64_t width = 0;
  std::uint64_t height = 1;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') continue;
    const auto parts = Split(line);
    if (parts.empty()) continue;
    std::string key = parts[0];
    std::transform(key.begin(), key.end(), key.begin(), ::toupper);
    if (key == "FIELDS") {
      header.fields.assign(parts.begin() + 1, parts.end());
    } else if (key == "SIZE") {
      for (std::size_t i = 1; i < parts.size(); ++i) header.sizes.push_back(std::stoi(parts[i]));
    } else if (key == "TYPE") {
      for (std::size_t i = 1; i < parts.size(); ++i) header.types.push_back(parts[i].at(0));
    } else if (key == "COUNT") {
      for (std::size_t i = 1; i < parts.size(); ++i) header.counts.push_back(std::stoi(parts[i]));
    } else if (key == "WIDTH" && parts.size() > 1) {
      width = std::stoull(parts[1]);
    } else if (key == "HEIGHT" && parts.size() > 1) {
      height = std::stoull(parts[1]);
    } else if (key == "POINTS" && parts.size() > 1) {
      header.points = std::stoull(parts[1]);
    } else if (key == "DATA" && parts.size() > 1) {
      header.data = parts[1];
      std::transform(header.data.begin(), header.data.end(), header.data.begin(), ::tolower);
      break;
    }
  }
  if (header.points == 0) header.points = width * height;
  if (header.counts.empty()) header.counts.assign(header.fields.size(), 1);
  if (header.fields.empty() || header.sizes.size() != header.fields.size() ||
      header.types.size() != header.fields.size() || header.counts.size() != header.fields.size() ||
      header.points == 0 || header.data.empty()) {
    throw std::runtime_error("INVALID_PCD: incomplete PCD header");
  }
  if (header.data != "ascii" && header.data != "binary") {
    throw std::runtime_error("INVALID_PCD: only ASCII and binary PCD are supported");
  }
  return header;
}

double ReadScalar(const char* data, char type, int size) {
  if (type == 'F' && size == 4) { float value; std::memcpy(&value, data, 4); return value; }
  if (type == 'F' && size == 8) { double value; std::memcpy(&value, data, 8); return value; }
  if (type == 'I' && size == 1) return *reinterpret_cast<const std::int8_t*>(data);
  if (type == 'I' && size == 2) { std::int16_t v; std::memcpy(&v, data, 2); return v; }
  if (type == 'I' && size == 4) { std::int32_t v; std::memcpy(&v, data, 4); return v; }
  if (type == 'U' && size == 1) return *reinterpret_cast<const std::uint8_t*>(data);
  if (type == 'U' && size == 2) { std::uint16_t v; std::memcpy(&v, data, 2); return v; }
  if (type == 'U' && size == 4) { std::uint32_t v; std::memcpy(&v, data, 4); return v; }
  throw std::runtime_error("INVALID_PCD: unsupported PCD field type");
}

std::vector<Point3> ReadPcd(const std::string& path, PcdHeader& header) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("INVALID_PCD: cannot open input file");
  header = ReadHeader(input);
  std::array<int, 3> field_indices{-1, -1, -1};
  for (std::size_t i = 0; i < header.fields.size(); ++i) {
    if (header.fields[i] == "x") field_indices[0] = static_cast<int>(i);
    if (header.fields[i] == "y") field_indices[1] = static_cast<int>(i);
    if (header.fields[i] == "z") field_indices[2] = static_cast<int>(i);
  }
  if (*std::min_element(field_indices.begin(), field_indices.end()) < 0) {
    throw std::runtime_error("INVALID_PCD: x/y/z fields are required");
  }
  std::vector<Point3> points;
  points.reserve(header.points);
  if (header.data == "ascii") {
    std::string line;
    while (points.size() < header.points && std::getline(input, line)) {
      const auto values = Split(line);
      if (values.size() < header.fields.size()) continue;
      points.push_back({std::stod(values[field_indices[0]]), std::stod(values[field_indices[1]]),
                        std::stod(values[field_indices[2]])});
    }
  } else {
    std::vector<int> offsets(header.fields.size());
    int stride = 0;
    for (std::size_t i = 0; i < header.fields.size(); ++i) {
      offsets[i] = stride;
      stride += header.sizes[i] * header.counts[i];
    }
    std::vector<char> buffer(static_cast<std::size_t>(stride));
    for (std::uint64_t i = 0; i < header.points; ++i) {
      if (!input.read(buffer.data(), stride)) throw std::runtime_error("INVALID_PCD: unexpected binary EOF");
      Point3 p;
      double* values[3] = {&p.x, &p.y, &p.z};
      for (int axis = 0; axis < 3; ++axis) {
        const int f = field_indices[axis];
        *values[axis] = ReadScalar(buffer.data() + offsets[f], header.types[f], header.sizes[f]);
      }
      points.push_back(p);
    }
  }
  if (points.size() != header.points) throw std::runtime_error("INVALID_PCD: point count mismatch");
  return points;
}

double Median(std::vector<double> values) {
  if (values.empty()) throw std::runtime_error("INVALID_PCD: cannot estimate ground from empty cloud");
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

double EstimateGround(const std::vector<Point3>& points, double min_x, double min_y) {
  struct Cell { int count{}; double min_z{std::numeric_limits<double>::infinity()}; };
  std::unordered_map<std::uint64_t, Cell> cells;
  cells.reserve(points.size() / 2 + 1);
  for (const auto& p : points) {
    const std::int64_t x = static_cast<std::int64_t>(std::floor((p.x - min_x) / 0.20));
    const std::int64_t y = static_cast<std::int64_t>(std::floor((p.y - min_y) / 0.20));
    const std::uint64_t key = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(y)) << 32) |
                              static_cast<std::uint32_t>(x);
    auto& cell = cells[key];
    ++cell.count;
    cell.min_z = std::min(cell.min_z, p.z);
  }
  std::vector<double> candidates;
  for (const auto& [_, cell] : cells) if (cell.count >= 2) candidates.push_back(cell.min_z);
  if (candidates.empty()) for (const auto& [_, cell] : cells) candidates.push_back(cell.min_z);
  std::sort(candidates.begin(), candidates.end());
  const std::size_t lo = candidates.size() / 100;
  const std::size_t hi = candidates.size() - 1 - candidates.size() / 100;
  std::unordered_map<long long, std::vector<double>> histogram;
  for (std::size_t i = lo; i <= hi; ++i) {
    histogram[static_cast<long long>(std::floor(candidates[i] / 0.05))].push_back(candidates[i]);
  }
  auto best = std::max_element(histogram.begin(), histogram.end(), [](const auto& a, const auto& b) {
    return a.second.size() < b.second.size();
  });
  return best == histogram.end() ? Median(candidates) : Median(best->second);
}

double Cross(const std::pair<double, double>& o, const std::pair<double, double>& a,
             const std::pair<double, double>& b) {
  return (a.first - o.first) * (b.second - o.second) -
         (a.second - o.second) * (b.first - o.first);
}

std::vector<std::pair<double, double>> ConvexHull(std::vector<std::pair<double, double>> points) {
  std::sort(points.begin(), points.end());
  points.erase(std::unique(points.begin(), points.end()), points.end());
  if (points.size() < 3) throw std::runtime_error("INVALID_PCD: fewer than three distinct XY points");
  std::vector<std::pair<double, double>> hull(points.size() * 2);
  std::size_t k = 0;
  for (const auto& p : points) {
    while (k >= 2 && Cross(hull[k - 2], hull[k - 1], p) <= 0.0) --k;
    hull[k++] = p;
  }
  for (std::size_t i = points.size() - 1, t = k + 1; i > 0; --i) {
    const auto& p = points[i - 1];
    while (k >= t && Cross(hull[k - 2], hull[k - 1], p) <= 0.0) --k;
    hull[k++] = p;
  }
  hull.resize(k - 1);
  return hull;
}

bool PointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& polygon) {
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto [xi, yi] = polygon[i];
    const auto [xj, yj] = polygon[j];
    const bool intersect = ((yi > y) != (yj > y)) &&
        (x < (xj - xi) * (y - yi) / ((yj - yi) == 0.0 ? 1e-12 : (yj - yi)) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}

std::size_t Index(const GridMeta& meta, int x, int y) {
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(meta.width) + static_cast<std::size_t>(x);
}

bool InBounds(const GridMeta& meta, int x, int y) {
  return x >= 0 && y >= 0 && x < meta.width && y < meta.height;
}

std::pair<int, int> WorldToGrid(const GridMeta& meta, double x, double y) {
  return {static_cast<int>(std::floor((x - meta.origin_x) / meta.resolution)),
          static_cast<int>(std::floor((y - meta.origin_y) / meta.resolution))};
}

std::pair<double, double> GridToWorld(const GridMeta& meta, int x, int y) {
  return {meta.origin_x + (x + 0.5) * meta.resolution,
          meta.origin_y + (y + 0.5) * meta.resolution};
}

bool Traversable(std::uint8_t cost, int max_cost = kInscribed - 1) {
  return static_cast<int>(cost) <= max_cost;
}

std::vector<std::pair<int, int>> RasterLine(int x0, int y0, int x1, int y1) {
  std::vector<std::pair<int, int>> result;
  int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  while (true) {
    result.emplace_back(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    const int twice = 2 * error;
    if (twice >= dy) { error += dy; x0 += sx; }
    if (twice <= dx) { error += dx; y0 += sy; }
  }
  return result;
}

std::vector<std::pair<int, int>> SupercoverLine(int x0, int y0, int x1, int y1) {
  std::vector<std::pair<int, int>> result{{x0, y0}};
  const int delta_x = x1 - x0;
  const int delta_y = y1 - y0;
  const int count_x = std::abs(delta_x);
  const int count_y = std::abs(delta_y);
  const int step_x = delta_x < 0 ? -1 : 1;
  const int step_y = delta_y < 0 ? -1 : 1;
  int crossed_x = 0;
  int crossed_y = 0;
  while (crossed_x < count_x || crossed_y < count_y) {
    const std::int64_t compare_x = static_cast<std::int64_t>(1 + 2 * crossed_x) * count_y;
    const std::int64_t compare_y = static_cast<std::int64_t>(1 + 2 * crossed_y) * count_x;
    if (compare_x == compare_y) {
      result.emplace_back(x0 + step_x, y0);
      result.emplace_back(x0, y0 + step_y);
      x0 += step_x;
      y0 += step_y;
      ++crossed_x;
      ++crossed_y;
      result.emplace_back(x0, y0);
    } else if (compare_x < compare_y) {
      x0 += step_x;
      ++crossed_x;
      result.emplace_back(x0, y0);
    } else {
      y0 += step_y;
      ++crossed_y;
      result.emplace_back(x0, y0);
    }
  }
  return result;
}

}  // namespace

BuildResult BuildMapFromPcd(const std::string& path, const MapConfig& config) {
  if (!(config.resolution > 0.0) || config.obstacle_min_height > config.obstacle_max_height ||
      config.min_points_per_cell < 1) {
    throw std::runtime_error("INVALID_CONFIG: invalid map build parameters");
  }
  PcdHeader header;
  auto raw = ReadPcd(path, header);
  BuildResult result;
  result.declared_points = header.points;
  result.data_encoding = header.data;
  result.min_bound = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity()};
  result.max_bound = {-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                      -std::numeric_limits<double>::infinity()};
  std::vector<Point3> points;
  points.reserve(raw.size());
  for (const auto& p : raw) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    points.push_back(p);
    result.min_bound.x = std::min(result.min_bound.x, p.x);
    result.min_bound.y = std::min(result.min_bound.y, p.y);
    result.min_bound.z = std::min(result.min_bound.z, p.z);
    result.max_bound.x = std::max(result.max_bound.x, p.x);
    result.max_bound.y = std::max(result.max_bound.y, p.y);
    result.max_bound.z = std::max(result.max_bound.z, p.z);
  }
  result.finite_points = points.size();
  if (points.empty()) throw std::runtime_error("INVALID_PCD: no finite XYZ points");
  result.meta.ground_z = EstimateGround(points, result.min_bound.x, result.min_bound.y);
  result.meta.resolution = config.resolution;
  result.meta.origin_x = std::floor(result.min_bound.x / config.resolution) * config.resolution - config.resolution;
  result.meta.origin_y = std::floor(result.min_bound.y / config.resolution) * config.resolution - config.resolution;
  result.meta.width = static_cast<int>(std::ceil((result.max_bound.x - result.meta.origin_x) / config.resolution)) + 2;
  result.meta.height = static_cast<int>(std::ceil((result.max_bound.y - result.meta.origin_y) / config.resolution)) + 2;
  const std::size_t cell_count = static_cast<std::size_t>(result.meta.width) * result.meta.height;
  std::vector<int> hits(cell_count, 0);
  std::vector<std::pair<double, double>> xy;
  xy.reserve(points.size());
  const double min_z = result.meta.ground_z + config.obstacle_min_height;
  const double max_z = result.meta.ground_z + config.obstacle_max_height;
  for (const auto& p : points) {
    xy.emplace_back(p.x, p.y);
    if (p.z < min_z || p.z > max_z) continue;
    ++result.obstacle_points;
    const auto [gx, gy] = WorldToGrid(result.meta, p.x, p.y);
    if (InBounds(result.meta, gx, gy)) ++hits[Index(result.meta, gx, gy)];
  }
  result.boundary = ConvexHull(std::move(xy));
  result.obstacles.assign(cell_count, kFree);
  for (std::size_t i = 0; i < hits.size(); ++i) {
    if (hits[i] >= config.min_points_per_cell) {
      result.obstacles[i] = kOccupied;
      ++result.occupied_cells;
    }
  }
  result.base_grid = ApplyBoundary(result.obstacles, result.meta, result.boundary);
  return result;
}

std::vector<std::uint8_t> ApplyBoundary(
    const std::vector<std::uint8_t>& obstacles, const GridMeta& meta,
    const std::vector<std::pair<double, double>>& boundary) {
  const std::size_t expected = static_cast<std::size_t>(meta.width) * meta.height;
  if (obstacles.size() != expected || boundary.size() < 3) {
    throw std::runtime_error("INVALID_CONFIG: invalid grid or boundary");
  }
  std::vector<std::uint8_t> grid(expected, kUnknown);
  for (int y = 0; y < meta.height; ++y) {
    for (int x = 0; x < meta.width; ++x) {
      const auto world = GridToWorld(meta, x, y);
      if (PointInPolygon(world.first, world.second, boundary)) {
        const auto i = Index(meta, x, y);
        grid[i] = obstacles[i] == kOccupied ? kOccupied : kFree;
      }
    }
  }
  return grid;
}

std::vector<std::uint8_t> MergeOverlay(
    const std::vector<std::uint8_t>& base, const std::vector<std::uint8_t>& overlay) {
  if (base.size() != overlay.size()) throw std::runtime_error("INVALID_CONFIG: overlay size mismatch");
  auto result = base;
  for (std::size_t i = 0; i < result.size(); ++i) {
    if (overlay[i] == kOverlayFree && base[i] != kUnknown) result[i] = kFree;
    else if (overlay[i] == kOverlayOccupied && base[i] != kUnknown) result[i] = kOccupied;
    else if (overlay[i] > kOverlayOccupied) throw std::runtime_error("INVALID_CONFIG: invalid overlay value");
  }
  return result;
}

std::vector<std::uint8_t> BuildCostmap(
    const std::vector<std::uint8_t>& final_grid, const GridMeta& meta,
    const CostConfig& config) {
  const std::size_t expected = static_cast<std::size_t>(meta.width) * meta.height;
  if (final_grid.size() != expected || config.hard_clearance < 0.0 ||
      config.inflation_radius < config.hard_clearance || config.cost_scaling <= 0.0) {
    throw std::runtime_error("INVALID_CONFIG: invalid costmap parameters");
  }
  std::vector<std::uint8_t> result(expected, kFree);
  const float infinity = std::numeric_limits<float>::infinity();
  std::vector<float> distance(expected, infinity);
  using QueueItem = std::pair<float, std::size_t>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
  for (std::size_t i = 0; i < expected; ++i) {
    if (final_grid[i] == kUnknown) result[i] = kUnknown;
    else if (final_grid[i] == kOccupied) {
      result[i] = kOccupied;
      distance[i] = 0.0F;
      queue.emplace(0.0F, i);
    }
  }
  const std::array<int, 8> dx{-1, 1, 0, 0, -1, -1, 1, 1};
  const std::array<int, 8> dy{0, 0, -1, 1, -1, 1, -1, 1};
  while (!queue.empty()) {
    const auto [dist, index] = queue.top(); queue.pop();
    if (dist != distance[index] || dist > config.inflation_radius) continue;
    const int x = static_cast<int>(index % meta.width);
    const int y = static_cast<int>(index / meta.width);
    for (int k = 0; k < 8; ++k) {
      const int nx = x + dx[k], ny = y + dy[k];
      if (!InBounds(meta, nx, ny)) continue;
      const auto ni = Index(meta, nx, ny);
      if (final_grid[ni] == kUnknown) continue;
      const float step = static_cast<float>(meta.resolution * ((dx[k] == 0 || dy[k] == 0) ? 1.0 : std::sqrt(2.0)));
      const float candidate = dist + step;
      if (candidate < distance[ni] && candidate <= config.inflation_radius) {
        distance[ni] = candidate;
        queue.emplace(candidate, ni);
      }
    }
  }
  for (std::size_t i = 0; i < expected; ++i) {
    if (result[i] == kUnknown || result[i] == kOccupied || !std::isfinite(distance[i])) continue;
    if (distance[i] <= config.hard_clearance) result[i] = kInscribed;
    else if (distance[i] <= config.inflation_radius) {
      const double value = 252.0 * std::exp(-config.cost_scaling * (distance[i] - config.hard_clearance));
      result[i] = static_cast<std::uint8_t>(std::clamp(std::lround(value), 1L, 252L));
    }
  }
  return result;
}

ValidationResult ValidateGrid(
    const std::vector<std::uint8_t>& final_grid,
    const std::vector<std::uint8_t>& costmap, const GridMeta& meta) {
  const std::size_t expected = static_cast<std::size_t>(meta.width) * meta.height;
  if (final_grid.size() != expected || costmap.size() != expected) {
    throw std::runtime_error("INVALID_CONFIG: grid size mismatch");
  }
  ValidationResult result;
  std::vector<std::uint8_t> visited(expected, 0);
  const std::array<int, 4> dx{-1, 1, 0, 0};
  const std::array<int, 4> dy{0, 0, -1, 1};
  for (std::size_t i = 0; i < expected; ++i) {
    if (final_grid[i] == kFree) ++result.free_cells;
    else if (final_grid[i] == kOccupied) ++result.occupied_cells;
    else ++result.unknown_cells;
    if (Traversable(costmap[i])) ++result.traversable_cells;
  }
  std::queue<std::size_t> queue;
  for (std::size_t seed = 0; seed < expected; ++seed) {
    if (visited[seed] || !Traversable(costmap[seed])) continue;
    ++result.connected_components;
    visited[seed] = 1; queue.push(seed);
    while (!queue.empty()) {
      const auto current = queue.front(); queue.pop();
      const int x = static_cast<int>(current % meta.width), y = static_cast<int>(current / meta.width);
      for (int k = 0; k < 4; ++k) {
        const int nx = x + dx[k], ny = y + dy[k];
        if (!InBounds(meta, nx, ny)) continue;
        const auto ni = Index(meta, nx, ny);
        if (!visited[ni] && Traversable(costmap[ni])) { visited[ni] = 1; queue.push(ni); }
      }
    }
  }
  return result;
}

PlanResult PlanPath(
    const std::vector<std::uint8_t>& costmap, const GridMeta& meta,
    std::pair<double, double> start, std::pair<double, double> goal,
    const PlanConfig& config) {
  PlanResult result;
  result.requested_start = start; result.requested_goal = goal;
  const std::size_t expected = static_cast<std::size_t>(meta.width) * meta.height;
  if (costmap.size() != expected || config.snap_radius < 0.0 || config.point_spacing <= 0.0 ||
      config.cost_weight < 0.0 || config.max_traversable_cost < 0 ||
      config.max_traversable_cost >= kInscribed) {
    result.error_code = "INVALID_CONFIG"; result.message = "invalid planning parameters"; return result;
  }
  auto snap = [&](std::pair<double, double> world, const std::string& outside_code,
                  const std::string& blocked_code, std::pair<int, int>& output, bool& snapped) -> bool {
    const auto raw = WorldToGrid(meta, world.first, world.second);
    if (!InBounds(meta, raw.first, raw.second)) {
      result.error_code = outside_code; result.message = "point is outside map bounds"; return false;
    }
    if (Traversable(costmap[Index(meta, raw.first, raw.second)], config.max_traversable_cost)) {
      output = raw; return true;
    }
    const int radius = static_cast<int>(std::ceil(config.snap_radius / meta.resolution));
    double best = std::numeric_limits<double>::infinity();
    for (int dy = -radius; dy <= radius; ++dy) for (int dx = -radius; dx <= radius; ++dx) {
      const int x = raw.first + dx, y = raw.second + dy;
      if (!InBounds(meta, x, y) ||
          !Traversable(costmap[Index(meta, x, y)], config.max_traversable_cost)) continue;
      const double distance = std::hypot(dx * meta.resolution, dy * meta.resolution);
      if (distance <= config.snap_radius && distance < best) { best = distance; output = {x, y}; }
    }
    if (!std::isfinite(best)) { result.error_code = blocked_code; result.message = "no traversable cell within snap radius"; return false; }
    snapped = true; return true;
  };
  std::pair<int, int> start_cell, goal_cell;
  if (!snap(start, "START_OUTSIDE", "START_BLOCKED", start_cell, result.start_snapped)) return result;
  if (!snap(goal, "GOAL_OUTSIDE", "GOAL_BLOCKED", goal_cell, result.goal_snapped)) return result;
  result.actual_start = GridToWorld(meta, start_cell.first, start_cell.second);
  result.actual_goal = GridToWorld(meta, goal_cell.first, goal_cell.second);

  struct Item {
    std::int64_t f;
    std::size_t index;
    bool operator<(const Item& other) const {
      if (f != other.f) return f > other.f;
      return index > other.index;
    }
  };
  constexpr std::int64_t kOrthogonalCost = 100000;
  constexpr std::int64_t kDiagonalCost = 141421;
  constexpr std::int64_t kWeightScale = 1000;
  constexpr std::int64_t kCostDenominator = 252 * kWeightScale;
  const std::int64_t scaled_weight = std::llround(config.cost_weight * kWeightScale);
  const auto step_cost = [&](int step_x, int step_y, std::uint8_t cell_cost) {
    const std::int64_t base = (step_x == 0 || step_y == 0) ? kOrthogonalCost : kDiagonalCost;
    const std::int64_t numerator = kCostDenominator + scaled_weight * cell_cost;
    return (base * numerator + kCostDenominator / 2) / kCostDenominator;
  };
  const std::int64_t inf = std::numeric_limits<std::int64_t>::max() / 4;
  std::vector<std::int64_t> g(expected, inf);
  std::vector<std::int64_t> parent(expected, -1);
  std::vector<std::uint8_t> closed(expected, 0);
  std::priority_queue<Item> open;
  const auto start_index = Index(meta, start_cell.first, start_cell.second);
  const auto goal_index = Index(meta, goal_cell.first, goal_cell.second);
  auto heuristic = [&](int x, int y) -> std::int64_t {
    const std::int64_t delta_x = std::abs(x - goal_cell.first);
    const std::int64_t delta_y = std::abs(y - goal_cell.second);
    const std::int64_t diagonal = std::min(delta_x, delta_y);
    return diagonal * kDiagonalCost + (std::max(delta_x, delta_y) - diagonal) * kOrthogonalCost;
  };
  g[start_index] = 0; open.push({heuristic(start_cell.first, start_cell.second), start_index});
  const std::array<int, 8> dx{-1, 1, 0, 0, -1, -1, 1, 1};
  const std::array<int, 8> dy{0, 0, -1, 1, -1, 1, -1, 1};
  while (!open.empty()) {
    const auto current = open.top(); open.pop();
    if (closed[current.index]) continue;
    closed[current.index] = 1; ++result.expanded_nodes;
    if (current.index == goal_index) break;
    const int x = static_cast<int>(current.index % meta.width), y = static_cast<int>(current.index / meta.width);
    for (int k = 0; k < 8; ++k) {
      const int nx = x + dx[k], ny = y + dy[k];
      if (!InBounds(meta, nx, ny)) continue;
      const auto ni = Index(meta, nx, ny);
      if (!Traversable(costmap[ni], config.max_traversable_cost) || closed[ni]) continue;
      if (dx[k] != 0 && dy[k] != 0 &&
          (!Traversable(costmap[Index(meta, x + dx[k], y)], config.max_traversable_cost) ||
           !Traversable(costmap[Index(meta, x, y + dy[k])], config.max_traversable_cost))) continue;
      const std::int64_t candidate = g[current.index] + step_cost(dx[k], dy[k], costmap[ni]);
      if (candidate < g[ni]) {
        g[ni] = candidate; parent[ni] = static_cast<std::int64_t>(current.index);
        open.push({candidate + heuristic(nx, ny), ni});
      }
    }
  }
  if (!closed[goal_index]) { result.error_code = "NO_PATH"; result.message = "no path found"; return result; }
  std::vector<std::pair<int, int>> dense;
  for (std::int64_t at = static_cast<std::int64_t>(goal_index); at >= 0; at = parent[at]) {
    dense.emplace_back(static_cast<int>(at % meta.width), static_cast<int>(at / meta.width));
    if (static_cast<std::size_t>(at) == start_index) break;
  }
  std::reverse(dense.begin(), dense.end());
  std::vector<std::int64_t> cumulative(dense.size(), 0);
  for (std::size_t i = 1; i < dense.size(); ++i) {
    const auto cell = dense[i];
    cumulative[i] = cumulative[i - 1] + step_cost(
        cell.first - dense[i - 1].first, cell.second - dense[i - 1].second,
        costmap[Index(meta, cell.first, cell.second)]);
  }
  auto line_cost = [&](const std::pair<int, int>& a, const std::pair<int, int>& b, bool& clear) {
    const auto touched = SupercoverLine(a.first, a.second, b.first, b.second);
    clear = true;
    for (const auto& [x, y] : touched) {
      if (!InBounds(meta, x, y) ||
          !Traversable(costmap[Index(meta, x, y)], config.max_traversable_cost)) {
        clear = false;
        return std::int64_t{0};
      }
    }
    const auto line = RasterLine(a.first, a.second, b.first, b.second);
    std::int64_t total = 0;
    for (std::size_t i = 0; i < line.size(); ++i) {
      const auto [x, y] = line[i];
      if (i > 0) {
        total += step_cost(x - line[i - 1].first, y - line[i - 1].second,
                           costmap[Index(meta, x, y)]);
      }
    }
    return total;
  };
  std::vector<std::pair<int, int>> sparse{dense.front()};
  for (std::size_t i = 0; i + 1 < dense.size();) {
    std::size_t best = i + 1;
    for (std::size_t j = dense.size() - 1; j > i + 1; --j) {
      bool clear = false; const std::int64_t direct = line_cost(dense[i], dense[j], clear);
      if (clear && direct <= cumulative[j] - cumulative[i]) { best = j; break; }
    }
    sparse.push_back(dense[best]); i = best;
  }
  std::vector<std::pair<double, double>> polyline;
  for (const auto& cell : sparse) polyline.push_back(GridToWorld(meta, cell.first, cell.second));
  result.points.push_back(polyline.front());
  for (std::size_t i = 1; i < polyline.size(); ++i) {
    auto a = polyline[i - 1], b = polyline[i];
    const double segment = std::hypot(b.first - a.first, b.second - a.second);
    result.length_m += segment;
    double offset = config.point_spacing;
    while (offset < segment - 1e-9) {
      const double t = offset / segment;
      result.points.emplace_back(a.first + t * (b.first - a.first), a.second + t * (b.second - a.second));
      offset += config.point_spacing;
    }
    if (std::hypot(result.points.back().first - b.first, result.points.back().second - b.second) > 1e-9) {
      result.points.push_back(b);
    }
  }
  result.total_cost = static_cast<double>(g[goal_index]) * meta.resolution / kOrthogonalCost;
  result.ok = true; result.message = "path found";
  return result;
}

}  // namespace rmp
