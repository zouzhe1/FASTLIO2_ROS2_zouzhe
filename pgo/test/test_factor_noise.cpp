#include <gtest/gtest.h>

#include <cmath>

#include "pgo/factor_noise.h"
#include "pgo/keyframe_admission.h"

TEST(FactorNoise, KeepsRotationAndTranslationUnitsSeparate)
{
  pgo::FactorNoiseConfig config;
  config.odometry_rotation_variance = 1e-6;
  config.odometry_translation_variance = 1e-4;

  const auto variances = pgo::odometryVariances(config);

  for (size_t i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(variances[i], 1e-6);
  for (size_t i = 3; i < 6; ++i) EXPECT_DOUBLE_EQ(variances[i], 1e-4);
}

TEST(FactorNoise, ClampsLoopScoreAndUsesSeparateScales)
{
  pgo::FactorNoiseConfig config;
  config.loop_rotation_scale = 0.1;
  config.loop_translation_scale = 1.0;
  config.loop_score_min = 1e-4;
  config.loop_score_max = 0.25;

  const auto low = pgo::loopVariances(-1.0, config);
  const auto high = pgo::loopVariances(10.0, config);

  EXPECT_DOUBLE_EQ(low[0], 1e-5);
  EXPECT_DOUBLE_EQ(low[3], 1e-4);
  EXPECT_DOUBLE_EQ(high[0], 0.025);
  EXPECT_DOUBLE_EQ(high[3], 0.25);
}

TEST(KeyframeAdmission, AdmitsOnlyMeaningfulMotion)
{
  pgo::KeyframeAdmission gate({1.0, 10.0});
  pgo::AdmissionPose first;
  EXPECT_TRUE(gate.shouldAdmit(first));
  gate.accept(first);

  pgo::AdmissionPose small;
  small.x = 0.2;
  small.qw = std::cos(2.5 * pgo::kPi / 180.0);
  small.qz = std::sin(2.5 * pgo::kPi / 180.0);
  EXPECT_FALSE(gate.shouldAdmit(small));

  pgo::AdmissionPose translated = small;
  translated.x = 1.1;
  EXPECT_TRUE(gate.shouldAdmit(translated));

  pgo::AdmissionPose rotated = small;
  rotated.qw = std::cos(5.5 * pgo::kPi / 180.0);
  rotated.qz = std::sin(5.5 * pgo::kPi / 180.0);
  EXPECT_TRUE(gate.shouldAdmit(rotated));
}
