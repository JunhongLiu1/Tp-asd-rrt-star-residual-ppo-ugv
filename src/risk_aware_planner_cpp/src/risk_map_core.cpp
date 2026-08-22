#include "risk_aware_planner_cpp/risk_map_core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace risk_aware_planner_cpp
{
RiskMapCore::RiskMapCore(RiskMapConfig config) : config_(config)
{
  if (config_.update_radius_m <= 0.0 || config_.sigma_m <= 0.0 ||
    config_.filter_alpha <= 0.0 || config_.filter_alpha > 1.0 ||
    config_.confidence_gain <= 0.0 || config_.confidence_gain > 1.0 ||
    config_.confidence_decay_per_sec < 0.0) {
    throw std::invalid_argument("invalid risk map configuration");
  }
}
bool RiskMapCore::resetGeometry(const RiskMapGeometry & geometry)
{
  if (geometry.width == 0 || geometry.height == 0 || geometry.resolution <= 0.0) return false;
  const bool changed = geometry.width != geometry_.width || geometry.height != geometry_.height ||
    std::abs(geometry.resolution - geometry_.resolution) > 1e-12 ||
    std::abs(geometry.origin_x - geometry_.origin_x) > 1e-12 ||
    std::abs(geometry.origin_y - geometry_.origin_y) > 1e-12 ||
    std::abs(geometry.origin_yaw - geometry_.origin_yaw) > 1e-12;
  if (changed) {
    geometry_ = geometry;
    dose_rate_.assign(static_cast<std::size_t>(geometry.width) * geometry.height, 0.0F);
    confidence_.assign(dose_rate_.size(), 0.0F);
    ++version_;  // Never reset: versions remain globally monotonic across geometry changes.
    updated_at_sec_ = -1.0;
  }
  return changed;
}
bool RiskMapCore::worldToCell(double x, double y, int & column, int & row) const
{
  const double dx = x - geometry_.origin_x, dy = y - geometry_.origin_y;
  const double lx = std::cos(geometry_.origin_yaw) * dx + std::sin(geometry_.origin_yaw) * dy;
  const double ly = -std::sin(geometry_.origin_yaw) * dx + std::cos(geometry_.origin_yaw) * dy;
  column = static_cast<int>(std::floor(lx / geometry_.resolution));
  row = static_cast<int>(std::floor(ly / geometry_.resolution));
  return column >= 0 && row >= 0 && column < static_cast<int>(geometry_.width) &&
    row < static_cast<int>(geometry_.height);
}
void RiskMapCore::decayTo(double timestamp_sec)
{
  if (!std::isfinite(timestamp_sec) || updated_at_sec_ < 0.0 || timestamp_sec <= updated_at_sec_) return;
  const float factor = static_cast<float>(std::exp(
    -config_.confidence_decay_per_sec * (timestamp_sec - updated_at_sec_)));
  for (float & value : confidence_) value *= factor;
}
bool RiskMapCore::applyMeasurement(double x, double y, double dose, double timestamp_sec)
{
  if (dose_rate_.empty() || !std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(dose) || dose < 0.0 || !std::isfinite(timestamp_sec) ||
    (updated_at_sec_ >= 0.0 && timestamp_sec < updated_at_sec_)) return false;
  int cx = 0, cy = 0;
  if (!worldToCell(x, y, cx, cy)) return false;
  decayTo(timestamp_sec);
  const int radius = static_cast<int>(std::ceil(config_.update_radius_m / geometry_.resolution));
  const double sigma2 = config_.sigma_m * config_.sigma_m;
  bool changed = false;
  for (int dy = -radius; dy <= radius; ++dy) for (int dx = -radius; dx <= radius; ++dx) {
    const int xcell = cx + dx, ycell = cy + dy;
    if (xcell < 0 || ycell < 0 || xcell >= static_cast<int>(geometry_.width) ||
      ycell >= static_cast<int>(geometry_.height)) continue;
    const double distance2 = (dx * dx + dy * dy) * geometry_.resolution * geometry_.resolution;
    if (distance2 > config_.update_radius_m * config_.update_radius_m) continue;
    const float kernel = static_cast<float>(std::exp(-distance2 / (2.0 * sigma2)));
    const std::size_t i = static_cast<std::size_t>(ycell) * geometry_.width + xcell;
    const float measured = static_cast<float>(dose) * kernel;
    const float alpha = confidence_[i] <= 0.0F ? 1.0F : static_cast<float>(config_.filter_alpha);
    dose_rate_[i] = (1.0F - alpha) * dose_rate_[i] + alpha * measured;
    confidence_[i] = std::min(1.0F, confidence_[i] + static_cast<float>(config_.confidence_gain) * kernel);
    changed = true;
  }
  if (changed) {++version_; updated_at_sec_ = timestamp_sec;}
  return changed;
}
double RiskMapCore::coverage() const
{
  if (confidence_.empty()) return 0.0;
  return static_cast<double>(std::count_if(confidence_.begin(), confidence_.end(),
    [](float value) {return value > 0.0F;})) / confidence_.size();
}
}  // namespace risk_aware_planner_cpp
