#include "pgo/loop_verifier.h"

#include <algorithm>
#include <stdexcept>

namespace pgo
{

LoopVerifier::LoopVerifier(LoopVerifierConfig config) : config_(config)
{
  if (config_.reject_count_to_blacklist == 0 || config_.blacklist_seconds <= 0) {
    throw std::invalid_argument("invalid loop verifier bounds");
  }
}

LoopVerifier::Pair LoopVerifier::pairFor(const LoopEvidence & evidence) const
{
  return std::minmax(evidence.source_id, evidence.target_id);
}

LoopDecision LoopVerifier::reject(
  const Pair & pair, const std::string & reason, double now_seconds)
{
  auto & rejection = rejections_[pair];
  if (++rejection.count >= config_.reject_count_to_blacklist) {
    rejection.blacklist_until = now_seconds + config_.blacklist_seconds;
  }
  return {false, reason};
}

LoopDecision LoopVerifier::verify(const LoopEvidence & evidence, double now_seconds)
{
  const auto pair = pairFor(evidence);
  auto rejection = rejections_.find(pair);
  if (rejection != rejections_.end() && now_seconds < rejection->second.blacklist_until) {
    return {false, "blacklisted"};
  }
  if (evidence.source_id == evidence.target_id) return reject(pair, "same_keyframe", now_seconds);
  if (evidence.descriptor_score > config_.max_descriptor_score) {
    return reject(pair, "descriptor_score", now_seconds);
  }
  if (evidence.second_descriptor_score - evidence.descriptor_score <
    config_.minimum_descriptor_margin) return reject(pair, "ambiguous", now_seconds);
  if (!evidence.geometry_verified) return reject(pair, "geometry", now_seconds);
  if (evidence.geometric_score > config_.max_geometric_score) {
    return reject(pair, "geometric_score", now_seconds);
  }
  if (evidence.translation_correction > config_.max_translation_correction ||
    evidence.rotation_correction_deg > config_.max_rotation_correction_deg) {
    return reject(pair, "correction_limit", now_seconds);
  }
  rejections_.erase(pair);
  return {true, "ok"};
}

}  // namespace pgo
