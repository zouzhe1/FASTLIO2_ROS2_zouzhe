#include "localizer/global_recovery.h"

#include <algorithm>
#include <stdexcept>

namespace localizer
{

GlobalRecovery::GlobalRecovery(GlobalRecoveryConfig config) : config_(config)
{
  if (config_.top_k == 0 || config_.total_deadline_ms <= 0 ||
    config_.minimum_score_margin < 0 || config_.confirmation_frames == 0) {
    throw std::invalid_argument("invalid global recovery bounds");
  }
}

RecoverySelection GlobalRecovery::select(
  const std::vector<RecoveryCandidate> & candidates, const std::string & active_level,
  double elapsed_ms, bool cancelled) const
{
  if (cancelled) return {false, 0, 1.0, 0.0, "cancelled"};
  if (elapsed_ms > config_.total_deadline_ms) {
    return {false, 0, 1.0, 0.0, "deadline_exceeded"};
  }
  std::vector<RecoveryCandidate> verified;
  for (std::size_t i = 0; i < std::min(config_.top_k, candidates.size()); ++i) {
    if ((active_level.empty() || candidates[i].level_id == active_level) &&
      candidates[i].geometrically_verified) {
      verified.push_back(candidates[i]);
    }
  }
  if (verified.empty()) return {false, 0, 1.0, 0.0, "no_verified_candidate"};
  std::sort(verified.begin(), verified.end(), [](const auto & a, const auto & b) {
    return a.geometric_score < b.geometric_score;
  });
  const double margin = verified.size() > 1 ?
    verified[1].geometric_score - verified[0].geometric_score : 1.0;
  if (margin < config_.minimum_score_margin) {
    return {false, 0, verified[0].geometric_score, margin, "ambiguous"};
  }
  return {true, verified[0].id, verified[0].geometric_score, margin, "ok"};
}

bool GlobalRecovery::confirmMotionFrame(bool consistent)
{
  if (!consistent) consistent_frames_ = 0;
  else ++consistent_frames_;
  return consistent_frames_ >= config_.confirmation_frames;
}

void GlobalRecovery::resetConfirmation() {consistent_frames_ = 0;}

}  // namespace localizer
