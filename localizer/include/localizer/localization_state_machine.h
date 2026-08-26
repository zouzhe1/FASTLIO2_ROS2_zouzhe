#pragma once

#include <cstddef>

namespace localizer
{

enum class LocalizationHealth
{
  UNINITIALIZED,
  TRACKING,
  DEGRADED,
  LOST,
  RELOCALIZING,
  RECOVERING
};

struct LocalizationStateMachineConfig
{
  std::size_t degraded_after_failures = 2;
  std::size_t lost_after_failures = 5;
  double trusted_timeout_seconds = 5.0;
  std::size_t recovery_consistent_frames = 3;
};

class LocalizationStateMachine
{
public:
  explicit LocalizationStateMachine(LocalizationStateMachineConfig config = {});

  LocalizationHealth state() const;
  bool globalPoseValid() const;
  bool shouldPublishGlobalTf() const;
  double lastTrustedTime() const;

  void acceptTrackingResult(double now_seconds);
  void rejectTrackingResult(double now_seconds);
  void tick(double now_seconds);

  bool beginRelocalization();
  bool acceptRecoveryCandidate();
  bool confirmRecoveryFrame(bool motion_consistent, double now_seconds);
  void failRelocalization();

private:
  void enterLost();

  LocalizationStateMachineConfig config_;
  LocalizationHealth state_ = LocalizationHealth::UNINITIALIZED;
  std::size_t consecutive_failures_ = 0;
  std::size_t recovery_consistent_frames_ = 0;
  double last_trusted_time_ = -1.0;
};

}  // namespace localizer
