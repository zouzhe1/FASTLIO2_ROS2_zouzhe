#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

namespace pgo
{

constexpr double kPi = 3.14159265358979323846;

struct AdmissionThresholds
{
  double translation_m{1.0};
  double rotation_deg{10.0};
};

struct AdmissionPose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qw{1.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
};

class KeyframeAdmission
{
public:
  explicit KeyframeAdmission(AdmissionThresholds thresholds)
  : thresholds_(thresholds) {}

  bool shouldAdmit(const AdmissionPose & pose) const
  {
    if (!last_pose_.has_value()) {
      return true;
    }
    const double dx = pose.x - last_pose_->x;
    const double dy = pose.y - last_pose_->y;
    const double dz = pose.z - last_pose_->z;
    const double translation = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double dot = std::abs(
      pose.qw * last_pose_->qw + pose.qx * last_pose_->qx +
      pose.qy * last_pose_->qy + pose.qz * last_pose_->qz);
    const double rotation_deg = 2.0 * std::acos(std::clamp(dot, 0.0, 1.0)) * 180.0 / kPi;
    return translation > thresholds_.translation_m || rotation_deg > thresholds_.rotation_deg;
  }

  void accept(const AdmissionPose & pose) {last_pose_ = pose;}

private:
  AdmissionThresholds thresholds_;
  std::optional<AdmissionPose> last_pose_;
};

}  // namespace pgo
