#include "localizer/localization_state_machine.h"

#include <algorithm>
#include <cmath>

namespace localizer
{

LocalizationStateMachine::LocalizationStateMachine(LocalizationStateMachineConfig config)
: config_(config)
{
  config_.degraded_after_failures = std::max<std::size_t>(1U, config_.degraded_after_failures);
  config_.lost_after_failures = std::max(
    config_.degraded_after_failures, config_.lost_after_failures);
  config_.trusted_timeout_seconds = std::max(0.0, config_.trusted_timeout_seconds);
  config_.recovery_consistent_frames = std::max<std::size_t>(
    1U, config_.recovery_consistent_frames);
}

LocalizationHealth LocalizationStateMachine::state() const
{
  return state_;
}

bool LocalizationStateMachine::globalPoseValid() const
{
  return last_trusted_time_ >= 0.0 &&
         (state_ == LocalizationHealth::TRACKING || state_ == LocalizationHealth::DEGRADED);
}

bool LocalizationStateMachine::shouldPublishGlobalTf() const
{
  return globalPoseValid();
}

double LocalizationStateMachine::lastTrustedTime() const
{
  return last_trusted_time_;
}

void LocalizationStateMachine::acceptTrackingResult(double now_seconds)
{
  if (!std::isfinite(now_seconds)) {
    return;
  }
  state_ = LocalizationHealth::TRACKING;
  consecutive_failures_ = 0;
  recovery_consistent_frames_ = 0;
  last_trusted_time_ = now_seconds;
}

void LocalizationStateMachine::rejectTrackingResult(double now_seconds)
{
  if (state_ != LocalizationHealth::TRACKING && state_ != LocalizationHealth::DEGRADED) {
    return;
  }
  ++consecutive_failures_;
  if (consecutive_failures_ >= config_.lost_after_failures) {
    enterLost();
    return;
  }
  if (consecutive_failures_ >= config_.degraded_after_failures) {
    state_ = LocalizationHealth::DEGRADED;
  }
  tick(now_seconds);
}

void LocalizationStateMachine::tick(double now_seconds)
{
  if (!std::isfinite(now_seconds) || last_trusted_time_ < 0.0) {
    return;
  }
  if ((state_ == LocalizationHealth::TRACKING || state_ == LocalizationHealth::DEGRADED) &&
    now_seconds - last_trusted_time_ > config_.trusted_timeout_seconds)
  {
    enterLost();
  }
}

bool LocalizationStateMachine::beginRelocalization()
{
  if (state_ != LocalizationHealth::UNINITIALIZED &&
    state_ != LocalizationHealth::TRACKING && state_ != LocalizationHealth::LOST &&
    state_ != LocalizationHealth::DEGRADED)
  {
    return false;
  }
  state_ = LocalizationHealth::RELOCALIZING;
  consecutive_failures_ = 0;
  recovery_consistent_frames_ = 0;
  return true;
}

bool LocalizationStateMachine::acceptRecoveryCandidate()
{
  if (state_ != LocalizationHealth::RELOCALIZING) {
    return false;
  }
  state_ = LocalizationHealth::RECOVERING;
  recovery_consistent_frames_ = 0;
  return true;
}

bool LocalizationStateMachine::confirmRecoveryFrame(
  bool motion_consistent, double now_seconds)
{
  if (state_ != LocalizationHealth::RECOVERING || !std::isfinite(now_seconds)) {
    return false;
  }
  if (!motion_consistent) {
    recovery_consistent_frames_ = 0;
    return false;
  }
  ++recovery_consistent_frames_;
  if (recovery_consistent_frames_ < config_.recovery_consistent_frames) {
    return false;
  }
  acceptTrackingResult(now_seconds);
  return true;
}

void LocalizationStateMachine::failRelocalization()
{
  enterLost();
}

void LocalizationStateMachine::enterLost()
{
  state_ = LocalizationHealth::LOST;
  consecutive_failures_ = 0;
  recovery_consistent_frames_ = 0;
}

}  // namespace localizer
