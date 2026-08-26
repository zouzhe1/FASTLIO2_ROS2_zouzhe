#pragma once

#include <cstddef>
#include <string>

namespace localizer
{

enum class QualityMode {TRACKING, RECOVERY};

struct QualityThresholds
{
  double tracking_max_rmse{0.20};
  std::size_t tracking_min_inliers{100};
  double tracking_min_inlier_ratio{0.30};
  double tracking_min_overlap{0.30};
  double tracking_max_condition{1e6};
  double tracking_max_translation{2.0};
  double tracking_max_rotation_deg{15.0};
  double recovery_max_rmse{0.15};
  std::size_t recovery_min_inliers{200};
  double recovery_min_inlier_ratio{0.50};
  double recovery_min_overlap{0.50};
  double recovery_max_condition{1e5};
  double recovery_max_translation{50.0};
  double recovery_max_rotation_deg{180.0};
  double max_elapsed_ms{80.0};
  double min_ambiguity_margin{0.15};
  double max_predicted_motion_error{1.0};
};

struct QualityInput
{
  bool converged{false};
  double rmse{0.0};
  std::size_t inliers{0};
  double inlier_ratio{0.0};
  double overlap{0.0};
  double hessian_condition{0.0};
  double correction_translation{0.0};
  double correction_rotation_deg{0.0};
  double elapsed_ms{0.0};
  bool tiles_complete{false};
  double ambiguity_margin{0.0};
  double predicted_motion_error{0.0};
};

struct QualityDecision
{
  bool accepted{false};
  bool requires_recovery_confirmation{false};
  std::string reason;
};

class RegistrationQualityGate
{
public:
  explicit RegistrationQualityGate(QualityThresholds thresholds);
  QualityDecision evaluate(const QualityInput & input, QualityMode mode) const;

private:
  QualityThresholds thresholds_;
};

}  // namespace localizer
