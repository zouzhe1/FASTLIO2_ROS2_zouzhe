#include "localizer/registration_quality_gate.h"

#include <cmath>

namespace localizer
{

RegistrationQualityGate::RegistrationQualityGate(QualityThresholds thresholds)
: thresholds_(thresholds) {}

QualityDecision RegistrationQualityGate::evaluate(
  const QualityInput & input, QualityMode mode) const
{
  auto reject = [](const char * reason) {return QualityDecision{false, false, reason};};
  if (!input.converged) return reject("not_converged");
  if (!std::isfinite(input.rmse) || !std::isfinite(input.hessian_condition)) {
    return reject("non_finite_metric");
  }
  if (!input.tiles_complete) return reject("tiles_incomplete");
  if (input.elapsed_ms > thresholds_.max_elapsed_ms) return reject("deadline");
  if (input.predicted_motion_error > thresholds_.max_predicted_motion_error) {
    return reject("motion_inconsistent");
  }
  const bool recovery = mode == QualityMode::RECOVERY;
  const double max_rmse = recovery ? thresholds_.recovery_max_rmse : thresholds_.tracking_max_rmse;
  const std::size_t min_inliers = recovery ? thresholds_.recovery_min_inliers :
    thresholds_.tracking_min_inliers;
  const double min_ratio = recovery ? thresholds_.recovery_min_inlier_ratio :
    thresholds_.tracking_min_inlier_ratio;
  const double min_overlap = recovery ? thresholds_.recovery_min_overlap :
    thresholds_.tracking_min_overlap;
  const double max_condition = recovery ? thresholds_.recovery_max_condition :
    thresholds_.tracking_max_condition;
  const double max_translation = recovery ? thresholds_.recovery_max_translation :
    thresholds_.tracking_max_translation;
  const double max_rotation = recovery ? thresholds_.recovery_max_rotation_deg :
    thresholds_.tracking_max_rotation_deg;
  if (input.rmse > max_rmse) return reject("rmse");
  if (input.inliers < min_inliers) return reject("inliers");
  if (input.inlier_ratio < min_ratio) return reject("inlier_ratio");
  if (input.overlap < min_overlap) return reject("overlap");
  if (input.hessian_condition > max_condition) return reject("conditioning");
  if (input.correction_translation > max_translation ||
    input.correction_rotation_deg > max_rotation) return reject("correction_jump");
  if (recovery && input.ambiguity_margin < thresholds_.min_ambiguity_margin) {
    return reject("ambiguous");
  }
  return {true, recovery, "ok"};
}

}  // namespace localizer
