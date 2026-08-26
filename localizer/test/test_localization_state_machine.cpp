#include <gtest/gtest.h>

#include "localizer/localization_state_machine.h"

using localizer::LocalizationHealth;
using localizer::LocalizationStateMachine;
using localizer::LocalizationStateMachineConfig;

TEST(LocalizationStateMachine, StartsWithoutAValidGlobalPose)
{
  LocalizationStateMachine machine;

  EXPECT_EQ(machine.state(), LocalizationHealth::UNINITIALIZED);
  EXPECT_FALSE(machine.globalPoseValid());
  EXPECT_FALSE(machine.shouldPublishGlobalTf());
}

TEST(LocalizationStateMachine, UsesFailureHysteresisBeforeLost)
{
  LocalizationStateMachineConfig config;
  config.degraded_after_failures = 2;
  config.lost_after_failures = 5;
  LocalizationStateMachine machine(config);
  machine.acceptTrackingResult(1.0);

  machine.rejectTrackingResult(2.0);
  EXPECT_EQ(machine.state(), LocalizationHealth::TRACKING);
  machine.rejectTrackingResult(3.0);
  EXPECT_EQ(machine.state(), LocalizationHealth::DEGRADED);
  EXPECT_TRUE(machine.shouldPublishGlobalTf());
  machine.rejectTrackingResult(4.0);
  machine.rejectTrackingResult(5.0);
  machine.rejectTrackingResult(6.0);

  EXPECT_EQ(machine.state(), LocalizationHealth::LOST);
  EXPECT_FALSE(machine.globalPoseValid());
  EXPECT_FALSE(machine.shouldPublishGlobalTf());
}

TEST(LocalizationStateMachine, BecomesLostWhenTrustedCorrectionExpires)
{
  LocalizationStateMachineConfig config;
  config.trusted_timeout_seconds = 5.0;
  LocalizationStateMachine machine(config);
  machine.acceptTrackingResult(10.0);

  machine.tick(14.9);
  EXPECT_EQ(machine.state(), LocalizationHealth::TRACKING);
  machine.tick(15.1);

  EXPECT_EQ(machine.state(), LocalizationHealth::LOST);
  EXPECT_FALSE(machine.shouldPublishGlobalTf());
}

TEST(LocalizationStateMachine, RequiresThreeConsistentRecoveryFrames)
{
  LocalizationStateMachineConfig config;
  config.recovery_consistent_frames = 3;
  LocalizationStateMachine machine(config);

  ASSERT_TRUE(machine.beginRelocalization());
  EXPECT_EQ(machine.state(), LocalizationHealth::RELOCALIZING);
  ASSERT_TRUE(machine.acceptRecoveryCandidate());
  EXPECT_EQ(machine.state(), LocalizationHealth::RECOVERING);

  EXPECT_FALSE(machine.confirmRecoveryFrame(true, 1.0));
  EXPECT_FALSE(machine.confirmRecoveryFrame(true, 2.0));
  EXPECT_TRUE(machine.confirmRecoveryFrame(true, 3.0));
  EXPECT_EQ(machine.state(), LocalizationHealth::TRACKING);
  EXPECT_TRUE(machine.globalPoseValid());
  EXPECT_TRUE(machine.shouldPublishGlobalTf());
}

TEST(LocalizationStateMachine, InconsistentRecoveryRestartsConfirmation)
{
  LocalizationStateMachine machine;
  ASSERT_TRUE(machine.beginRelocalization());
  ASSERT_TRUE(machine.acceptRecoveryCandidate());

  EXPECT_FALSE(machine.confirmRecoveryFrame(true, 1.0));
  EXPECT_FALSE(machine.confirmRecoveryFrame(false, 2.0));
  EXPECT_FALSE(machine.confirmRecoveryFrame(true, 3.0));
  EXPECT_FALSE(machine.confirmRecoveryFrame(true, 4.0));
  EXPECT_TRUE(machine.confirmRecoveryFrame(true, 5.0));
}

TEST(LocalizationStateMachine, ManualRelocalizationInvalidatesAFormerTrackingPose)
{
  LocalizationStateMachine machine;
  machine.acceptTrackingResult(1.0);
  ASSERT_TRUE(machine.globalPoseValid());

  EXPECT_TRUE(machine.beginRelocalization());
  EXPECT_EQ(machine.state(), LocalizationHealth::RELOCALIZING);
  EXPECT_FALSE(machine.globalPoseValid());
  EXPECT_FALSE(machine.shouldPublishGlobalTf());
}
