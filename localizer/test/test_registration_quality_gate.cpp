#include <gtest/gtest.h>

#include "localizer/registration_quality_gate.h"

namespace
{
localizer::QualityInput good()
{
  return {true, 0.05, 300, 0.7, 0.65, 100.0, 0.2, 2.0, 20.0, true, 0.3};
}
}  // namespace

TEST(RegistrationQualityGate, AcceptsGoodTrackingAndNamesEveryRejectedMetric)
{
  localizer::RegistrationQualityGate gate({});
  EXPECT_TRUE(gate.evaluate(good(), localizer::QualityMode::TRACKING).accepted);
  auto input = good(); input.rmse = 1.0;
  EXPECT_EQ(gate.evaluate(input, localizer::QualityMode::TRACKING).reason, "rmse");
  input = good(); input.inliers = 2;
  EXPECT_EQ(gate.evaluate(input, localizer::QualityMode::TRACKING).reason, "inliers");
  input = good(); input.overlap = 0.01;
  EXPECT_EQ(gate.evaluate(input, localizer::QualityMode::TRACKING).reason, "overlap");
  input = good(); input.hessian_condition = 1e12;
  EXPECT_EQ(gate.evaluate(input, localizer::QualityMode::TRACKING).reason, "conditioning");
  input = good(); input.correction_translation = 8.0;
  EXPECT_EQ(gate.evaluate(input, localizer::QualityMode::TRACKING).reason, "correction_jump");
  input = good(); input.elapsed_ms = 200.0;
  EXPECT_EQ(gate.evaluate(input, localizer::QualityMode::TRACKING).reason, "deadline");
  input = good(); input.tiles_complete = false;
  EXPECT_EQ(gate.evaluate(input, localizer::QualityMode::TRACKING).reason, "tiles_incomplete");
}

TEST(RegistrationQualityGate, RecoveryRequiresStrongerEvidenceAndAmbiguityMargin)
{
  localizer::RegistrationQualityGate gate({});
  auto input = good();
  input.ambiguity_margin = 0.01;
  EXPECT_EQ(gate.evaluate(input, localizer::QualityMode::RECOVERY).reason, "ambiguous");
  input.ambiguity_margin = 0.3;
  EXPECT_TRUE(gate.evaluate(input, localizer::QualityMode::RECOVERY).accepted);
}
