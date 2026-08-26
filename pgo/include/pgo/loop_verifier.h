#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>

namespace pgo
{

struct LoopVerifierConfig
{
  double max_descriptor_score{0.15};
  double minimum_descriptor_margin{0.15};
  double max_translation_correction{5.0};
  double max_rotation_correction_deg{45.0};
  std::size_t reject_count_to_blacklist{3};
  double blacklist_seconds{60.0};
  double max_geometric_score{0.15};
};

struct LoopEvidence
{
  std::uint64_t source_id{0};
  std::uint64_t target_id{0};
  double descriptor_score{1.0};
  double second_descriptor_score{1.0};
  bool geometry_verified{false};
  double geometric_score{1.0};
  double translation_correction{0.0};
  double rotation_correction_deg{0.0};
};

struct LoopDecision {bool accepted{false}; std::string reason;};

class LoopVerifier
{
public:
  explicit LoopVerifier(LoopVerifierConfig config);
  LoopDecision verify(const LoopEvidence & evidence, double now_seconds);

private:
  struct Rejection {std::size_t count{0}; double blacklist_until{0.0};};
  using Pair = std::pair<std::uint64_t, std::uint64_t>;
  Pair pairFor(const LoopEvidence & evidence) const;
  LoopDecision reject(const Pair & pair, const std::string & reason, double now_seconds);
  LoopVerifierConfig config_;
  std::map<Pair, Rejection> rejections_;
};

}  // namespace pgo
