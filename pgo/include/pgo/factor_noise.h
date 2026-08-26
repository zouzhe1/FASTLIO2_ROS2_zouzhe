#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace pgo
{

struct FactorNoiseConfig
{
  double odometry_rotation_variance{1e-6};
  double odometry_translation_variance{1e-4};
  double loop_rotation_scale{0.1};
  double loop_translation_scale{1.0};
  double loop_score_min{1e-4};
  double loop_score_max{0.25};
};

inline std::array<double, 6> odometryVariances(const FactorNoiseConfig & config)
{
  return {
    config.odometry_rotation_variance,
    config.odometry_rotation_variance,
    config.odometry_rotation_variance,
    config.odometry_translation_variance,
    config.odometry_translation_variance,
    config.odometry_translation_variance};
}

inline std::array<double, 6> loopVariances(
  double registration_score, const FactorNoiseConfig & config)
{
  const double finite_score = std::isfinite(registration_score) ? registration_score :
    config.loop_score_max;
  const double score = std::clamp(
    finite_score, config.loop_score_min, config.loop_score_max);
  const double rotation = score * config.loop_rotation_scale;
  const double translation = score * config.loop_translation_scale;
  return {rotation, rotation, rotation, translation, translation, translation};
}

}  // namespace pgo
