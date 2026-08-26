#include <cmath>
#include <deque>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "fastlio2/sensor_contract.h"

using fastlio2::SampleDisposition;
using fastlio2::SensorContract;
using fastlio2::SensorContractConfig;
using fastlio2::Vector3Sample;

TEST(SensorContract, ScalesAccelerationFromConfiguredSourceUnits)
{
  SensorContractConfig config;
  config.imu_acc_scale = 9.80665;
  SensorContract contract(config);

  const auto decision = contract.validateImu(
    1.0, Vector3Sample{1.0, -0.5, 0.25}, Vector3Sample{});

  ASSERT_EQ(decision.disposition, SampleDisposition::ACCEPTED);
  EXPECT_NEAR(decision.acceleration.x, 9.80665, 1e-9);
  EXPECT_NEAR(decision.acceleration.y, -4.903325, 1e-9);
  EXPECT_NEAR(decision.acceleration.z, 2.4516625, 1e-9);
}

TEST(SensorContract, SupportsAlreadyScaledSiAcceleration)
{
  SensorContractConfig config;
  config.imu_acc_scale = 1.0;
  SensorContract contract(config);

  const auto decision = contract.validateImu(
    1.0, Vector3Sample{0.0, 0.0, 9.81}, Vector3Sample{});

  ASSERT_EQ(decision.disposition, SampleDisposition::ACCEPTED);
  EXPECT_NEAR(decision.acceleration.z, 9.81, 1e-12);
}

TEST(SensorContract, RejectsNonFiniteAndSaturatedImuSamples)
{
  SensorContractConfig config;
  config.max_acceleration_mps2 = 50.0;
  config.max_angular_velocity_rps = 5.0;
  SensorContract contract(config);

  const auto non_finite = contract.validateImu(
    1.0, Vector3Sample{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
    Vector3Sample{});
  const auto saturated = contract.validateImu(
    2.0, Vector3Sample{60.0, 0.0, 0.0}, Vector3Sample{});

  EXPECT_EQ(non_finite.disposition, SampleDisposition::NON_FINITE);
  EXPECT_EQ(saturated.disposition, SampleDisposition::SATURATED);
  EXPECT_EQ(contract.diagnostics().non_finite_count, 1U);
  EXPECT_EQ(contract.diagnostics().saturation_count, 1U);
}

TEST(SensorContract, ResetsContinuityOnRegressionAndExcessiveGap)
{
  SensorContractConfig config;
  config.max_imu_gap_seconds = 0.02;
  SensorContract contract(config);

  EXPECT_EQ(
    contract.validateImu(1.0, Vector3Sample{}, Vector3Sample{}).disposition,
    SampleDisposition::ACCEPTED);
  const auto gap = contract.validateImu(1.05, Vector3Sample{}, Vector3Sample{});
  const auto regression = contract.validateImu(0.5, Vector3Sample{}, Vector3Sample{});

  EXPECT_EQ(gap.disposition, SampleDisposition::ACCEPTED_AFTER_GAP);
  EXPECT_TRUE(gap.reset_buffer);
  EXPECT_EQ(regression.disposition, SampleDisposition::TIMESTAMP_REGRESSION);
  EXPECT_TRUE(regression.reset_buffer);
  EXPECT_EQ(contract.diagnostics().imu_gap_count, 1U);
  EXPECT_EQ(contract.diagnostics().timestamp_regression_count, 1U);
}

TEST(SensorContract, RejectsEmptyLidarAndDetectsLidarGap)
{
  SensorContractConfig config;
  config.max_lidar_gap_seconds = 0.2;
  SensorContract contract(config);

  EXPECT_EQ(contract.validateLidar(1.0, 0).disposition, SampleDisposition::EMPTY);
  EXPECT_EQ(contract.validateLidar(2.0, 10).disposition, SampleDisposition::ACCEPTED);
  const auto gap = contract.validateLidar(2.3, 10);

  EXPECT_EQ(gap.disposition, SampleDisposition::ACCEPTED_AFTER_GAP);
  EXPECT_TRUE(gap.reset_buffer);
  EXPECT_EQ(contract.diagnostics().lidar_gap_count, 1U);
}

TEST(SensorContract, RequiresStationaryInitializationWhenConfigured)
{
  SensorContractConfig config;
  config.require_stationary_init = true;
  config.max_init_acc_stddev = 0.05;
  config.max_init_gyro_stddev = 0.01;
  SensorContract contract(config);

  const std::vector<Vector3Sample> stationary_acc(20, Vector3Sample{0.0, 0.0, 9.81});
  const std::vector<Vector3Sample> stationary_gyro(20, Vector3Sample{});
  auto moving_acc = stationary_acc;
  moving_acc.back().x = 1.0;

  EXPECT_TRUE(contract.isStationary(stationary_acc, stationary_gyro));
  EXPECT_FALSE(contract.isStationary(moving_acc, stationary_gyro));
}

TEST(SensorContract, TrimsBuffersByAgeAndCount)
{
  std::deque<double> imu_times{0.0, 0.5, 1.0, 1.5, 2.0};
  std::deque<int> lidar_frames{1, 2, 3, 4};

  const auto imu_removed = fastlio2::trimByAge(
    imu_times, 2.0, 1.0, [](double stamp) {return stamp;});
  const auto lidar_removed = fastlio2::trimByCount(lidar_frames, 2U);

  EXPECT_EQ(imu_removed, 2U);
  ASSERT_EQ(imu_times.size(), 3U);
  EXPECT_DOUBLE_EQ(imu_times.front(), 1.0);
  EXPECT_EQ(lidar_removed, 2U);
  EXPECT_EQ(lidar_frames.front(), 3);
}

TEST(SensorContract, ExtrinsicFingerprintIsStableAndSensitive)
{
  const std::vector<double> rotation{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  const std::vector<double> translation{0.0, 0.0, 0.0};
  auto changed_translation = translation;
  changed_translation[0] = 0.01;

  const auto first = fastlio2::fingerprintExtrinsics(rotation, translation, 9.80665);
  const auto second = fastlio2::fingerprintExtrinsics(rotation, translation, 9.80665);
  const auto changed = fastlio2::fingerprintExtrinsics(rotation, changed_translation, 9.80665);

  EXPECT_EQ(first, second);
  EXPECT_NE(first, changed);
  EXPECT_EQ(first.size(), 16U);
}
