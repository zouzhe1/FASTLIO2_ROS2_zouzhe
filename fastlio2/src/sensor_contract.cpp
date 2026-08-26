#include "fastlio2/sensor_contract.h"

#include <algorithm>
#include <array>
#include <iomanip>
#include <limits>
#include <sstream>

namespace fastlio2
{
namespace
{

bool isFinite(const Vector3Sample & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

double norm(const Vector3Sample & value)
{
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

double componentStddev(
  const std::vector<Vector3Sample> & values, double Vector3Sample::* component)
{
  if (values.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double mean = 0.0;
  for (const auto & value : values) {
    mean += value.*component;
  }
  mean /= static_cast<double>(values.size());

  double variance = 0.0;
  for (const auto & value : values) {
    const double residual = value.*component - mean;
    variance += residual * residual;
  }
  return std::sqrt(variance / static_cast<double>(values.size()));
}

double maximumComponentStddev(const std::vector<Vector3Sample> & values)
{
  return std::max({
    componentStddev(values, &Vector3Sample::x),
    componentStddev(values, &Vector3Sample::y),
    componentStddev(values, &Vector3Sample::z)});
}

}  // namespace

SensorContract::SensorContract(SensorContractConfig config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.imu_acc_scale) || config_.imu_acc_scale <= 0.0) {
    config_.imu_acc_scale = 9.80665;
  }
  config_.max_imu_gap_seconds = std::max(0.0, config_.max_imu_gap_seconds);
  config_.max_lidar_gap_seconds = std::max(0.0, config_.max_lidar_gap_seconds);
  config_.max_imu_buffer_seconds = std::max(0.0, config_.max_imu_buffer_seconds);
  config_.max_lidar_buffer_frames = std::max<std::size_t>(1U, config_.max_lidar_buffer_frames);
}

SampleDecision SensorContract::validateTimestamp(
  double timestamp, double max_gap_seconds, double & last_timestamp,
  std::uint32_t & gap_count)
{
  if (!std::isfinite(timestamp)) {
    ++diagnostics_.non_finite_count;
    return {SampleDisposition::NON_FINITE, false, {}};
  }

  if (last_timestamp >= 0.0 && timestamp <= last_timestamp) {
    ++diagnostics_.timestamp_regression_count;
    last_timestamp = timestamp;
    return {SampleDisposition::TIMESTAMP_REGRESSION, true, {}};
  }

  const bool has_gap = last_timestamp >= 0.0 &&
    timestamp - last_timestamp > max_gap_seconds;
  last_timestamp = timestamp;
  if (has_gap) {
    ++gap_count;
    return {SampleDisposition::ACCEPTED_AFTER_GAP, true, {}};
  }
  return {SampleDisposition::ACCEPTED, false, {}};
}

SampleDecision SensorContract::validateImu(
  double timestamp, const Vector3Sample & raw_acceleration,
  const Vector3Sample & angular_velocity)
{
  if (!isFinite(raw_acceleration) || !isFinite(angular_velocity)) {
    ++diagnostics_.non_finite_count;
    return {SampleDisposition::NON_FINITE, false, {}};
  }

  const Vector3Sample acceleration{
    raw_acceleration.x * config_.imu_acc_scale,
    raw_acceleration.y * config_.imu_acc_scale,
    raw_acceleration.z * config_.imu_acc_scale};
  if (norm(acceleration) > config_.max_acceleration_mps2 ||
    norm(angular_velocity) > config_.max_angular_velocity_rps)
  {
    ++diagnostics_.saturation_count;
    return {SampleDisposition::SATURATED, false, acceleration};
  }

  auto decision = validateTimestamp(
    timestamp, config_.max_imu_gap_seconds, last_imu_timestamp_,
    diagnostics_.imu_gap_count);
  decision.acceleration = acceleration;
  return decision;
}

SampleDecision SensorContract::validateLidar(double timestamp, std::size_t point_count)
{
  if (point_count == 0U) {
    ++diagnostics_.empty_lidar_count;
    return {SampleDisposition::EMPTY, false, {}};
  }
  return validateTimestamp(
    timestamp, config_.max_lidar_gap_seconds, last_lidar_timestamp_,
    diagnostics_.lidar_gap_count);
}

bool SensorContract::isStationary(
  const std::vector<Vector3Sample> & accelerations,
  const std::vector<Vector3Sample> & angular_velocities) const
{
  if (!config_.require_stationary_init) {
    return true;
  }
  if (accelerations.empty() || accelerations.size() != angular_velocities.size()) {
    return false;
  }
  return maximumComponentStddev(accelerations) <= config_.max_init_acc_stddev &&
         maximumComponentStddev(angular_velocities) <= config_.max_init_gyro_stddev;
}

const SensorContractConfig & SensorContract::config() const
{
  return config_;
}

const SensorDiagnostics & SensorContract::diagnostics() const
{
  return diagnostics_;
}

void SensorContract::resetContinuity()
{
  last_imu_timestamp_ = -1.0;
  last_lidar_timestamp_ = -1.0;
}

std::string fingerprintExtrinsics(
  const std::vector<double> & rotation, const std::vector<double> & translation,
  double imu_acc_scale)
{
  std::ostringstream canonical;
  canonical << std::hexfloat;
  for (double value : rotation) {
    canonical << value << ';';
  }
  for (double value : translation) {
    canonical << value << ';';
  }
  canonical << imu_acc_scale;

  std::uint64_t hash = 14695981039346656037ULL;
  for (const unsigned char value : canonical.str()) {
    hash ^= value;
    hash *= 1099511628211ULL;
  }

  std::ostringstream result;
  result << std::hex << std::setfill('0') << std::setw(16) << hash;
  return result.str();
}

}  // namespace fastlio2
