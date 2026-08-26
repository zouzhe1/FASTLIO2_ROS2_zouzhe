#include <gtest/gtest.h>

#include <cmath>

#include "localizer/registration_backend.h"

namespace
{
std::vector<Eigen::Vector3d> cloud()
{
  std::vector<Eigen::Vector3d> points;
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
      points.emplace_back(x * 0.3, y * 0.2, std::sin(x * 0.4) + y * 0.03);
    }
  }
  return points;
}
}  // namespace

TEST(SmallGicpBackend, RecoversKnownTranslationWithBoundedInput)
{
  localizer::RegistrationConfig config;
  config.voxel_resolution = 0.05;
  config.max_source_points = 100;
  config.max_iterations = 30;
  config.max_correspondence_distance = 1.0;
  config.deadline_ms = 1000.0;
  localizer::SmallGicpBackend backend(config);
  const auto target = cloud();
  backend.setTarget(target, 11);
  auto source = target;
  for (auto & point : source) point.x() -= 0.25;

  const auto result = backend.align(source, Eigen::Isometry3d::Identity());

  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.transform.translation().x(), 0.25, 0.04);
  EXPECT_GT(result.inliers, 20U);
  EXPECT_TRUE(std::isfinite(result.hessian_condition));
}

TEST(SmallGicpBackend, ReusesTargetAndRejectsEmptyOrDeadlineOverrun)
{
  localizer::RegistrationConfig config;
  config.voxel_resolution = 0.05;
  config.deadline_ms = 1000.0;
  localizer::SmallGicpBackend backend(config);
  backend.setTarget(cloud(), 5);
  backend.setTarget(cloud(), 5);
  EXPECT_EQ(backend.targetPreprocessCount(), 1U);
  EXPECT_FALSE(backend.align({}, Eigen::Isometry3d::Identity()).converged);

  config.deadline_ms = 0.0;
  localizer::SmallGicpBackend expired(config);
  expired.setTarget(cloud(), 1);
  const auto result = expired.align(cloud(), Eigen::Isometry3d::Identity());
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.reason, "deadline_exceeded");
}
