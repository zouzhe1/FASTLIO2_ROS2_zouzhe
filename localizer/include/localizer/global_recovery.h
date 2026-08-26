#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace localizer
{

struct GlobalRecoveryConfig
{
  std::size_t top_k{3};
  double total_deadline_ms{3000.0};
  double minimum_score_margin{0.15};
  std::size_t confirmation_frames{3};
};

struct RecoveryCandidate
{
  std::uint64_t id{0};
  std::string level_id;
  double descriptor_score{1.0};
  bool geometrically_verified{false};
  double geometric_score{1.0};
};

struct RecoverySelection
{
  bool accepted{false};
  std::uint64_t candidate_id{0};
  double score{1.0};
  double ambiguity_margin{0.0};
  std::string reason;
};

class GlobalRecovery
{
public:
  explicit GlobalRecovery(GlobalRecoveryConfig config);
  RecoverySelection select(
    const std::vector<RecoveryCandidate> & candidates, const std::string & active_level,
    double elapsed_ms, bool cancelled) const;
  bool confirmMotionFrame(bool consistent);
  void resetConfirmation();

private:
  GlobalRecoveryConfig config_;
  std::size_t consistent_frames_{0};
};

}  // namespace localizer
