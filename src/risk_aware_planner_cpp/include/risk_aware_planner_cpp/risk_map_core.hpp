#ifndef RISK_AWARE_PLANNER_CPP__RISK_MAP_CORE_HPP_
#define RISK_AWARE_PLANNER_CPP__RISK_MAP_CORE_HPP_

#include <cstdint>
#include <vector>

namespace risk_aware_planner_cpp
{
struct RiskMapGeometry
{
  unsigned int width{0}, height{0};
  double resolution{0.0}, origin_x{0.0}, origin_y{0.0}, origin_yaw{0.0};
};
struct RiskMapConfig
{
  double update_radius_m{2.5};
  double sigma_m{0.8};
  double filter_alpha{0.35};
  double confidence_gain{0.25};
  double confidence_decay_per_sec{0.002};
};
class RiskMapCore
{
public:
  explicit RiskMapCore(RiskMapConfig config = RiskMapConfig());
  bool resetGeometry(const RiskMapGeometry & geometry);
  bool applyMeasurement(double x, double y, double dose_rate_usv_h, double timestamp_sec);
  void decayTo(double timestamp_sec);
  const RiskMapGeometry & geometry() const {return geometry_;}
  const std::vector<float> & doseRate() const {return dose_rate_;}
  const std::vector<float> & confidence() const {return confidence_;}
  std::uint64_t version() const {return version_;}
  double updatedAt() const {return updated_at_sec_;}
  double coverage() const;
private:
  bool worldToCell(double x, double y, int & column, int & row) const;
  RiskMapConfig config_;
  RiskMapGeometry geometry_;
  std::vector<float> dose_rate_, confidence_;
  std::uint64_t version_{0};
  double updated_at_sec_{-1.0};
};
}  // namespace risk_aware_planner_cpp
#endif
