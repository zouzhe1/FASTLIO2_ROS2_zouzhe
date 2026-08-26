#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace fastlio2
{

struct Vector3Sample
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

enum class SampleDisposition
{
  ACCEPTED,
  ACCEPTED_AFTER_GAP,
  NON_FINITE,
  SATURATED,
  TIMESTAMP_REGRESSION,
  EMPTY
};

struct SensorContractConfig
{
  double imu_acc_scale = 9.80665;
  double max_imu_gap_seconds = 0.02;
  double max_lidar_gap_seconds = 0.2;
  double max_acceleration_mps2 = 200.0;
  double max_angular_velocity_rps = 20.0;
  double max_imu_buffer_seconds = 2.0;
  std::size_t max_lidar_buffer_frames = 3;
  bool require_stationary_init = true;
  double max_init_gyro_stddev = 0.02;
  double max_init_acc_stddev = 0.15;
};

struct SensorDiagnostics
{
  std::uint32_t imu_gap_count = 0;
  std::uint32_t lidar_gap_count = 0;
  std::uint32_t timestamp_regression_count = 0;
  std::uint32_t non_finite_count = 0;
  std::uint32_t saturation_count = 0;
  std::uint32_t empty_lidar_count = 0;
};

struct SampleDecision
{
  SampleDisposition disposition = SampleDisposition::NON_FINITE;
  bool reset_buffer = false;
  Vector3Sample acceleration;

  bool accepted() const
  {
    return disposition == SampleDisposition::ACCEPTED ||
           disposition == SampleDisposition::ACCEPTED_AFTER_GAP;
  }
};

class SensorContract
{
public:
  explicit SensorContract(SensorContractConfig config = {});

  SampleDecision validateImu(
    double timestamp, const Vector3Sample & raw_acceleration,
    const Vector3Sample & angular_velocity);
  SampleDecision validateLidar(double timestamp, std::size_t point_count);

  bool isStationary(
    const std::vector<Vector3Sample> & accelerations,
    const std::vector<Vector3Sample> & angular_velocities) const;

  const SensorContractConfig & config() const;
  const SensorDiagnostics & diagnostics() const;
  void resetContinuity();

private:
  SampleDecision validateTimestamp(
    double timestamp, double max_gap_seconds, double & last_timestamp,
    std::uint32_t & gap_count);

  SensorContractConfig config_;
  SensorDiagnostics diagnostics_;
  double last_imu_timestamp_ = -1.0;
  double last_lidar_timestamp_ = -1.0;
};

template<typename T, typename TimestampFn>
std::size_t trimByAge(
  std::deque<T> & buffer, double newest_timestamp, double max_age_seconds,
  TimestampFn timestamp_of)
{
  std::size_t removed = 0;
  while (!buffer.empty() &&
    newest_timestamp - timestamp_of(buffer.front()) > max_age_seconds)
  {
    buffer.pop_front();
    ++removed;
  }
  return removed;
}

template<typename T>
std::size_t trimByCount(std::deque<T> & buffer, std::size_t maximum_count)
{
  std::size_t removed = 0;
  while (buffer.size() > maximum_count) {
    buffer.pop_front();
    ++removed;
  }
  return removed;
}

std::string fingerprintExtrinsics(
  const std::vector<double> & rotation, const std::vector<double> & translation,
  double imu_acc_scale);

}  // namespace fastlio2
