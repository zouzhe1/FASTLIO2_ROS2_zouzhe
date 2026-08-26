#include "localizer/tf_policy.h"

#include <stdexcept>

namespace localizer
{

OperationalProfile parseOperationalProfile(const std::string & value)
{
  if (value == "mapping") {
    return OperationalProfile::MAPPING;
  }
  if (value == "localization") {
    return OperationalProfile::LOCALIZATION;
  }
  if (value == "maintenance") {
    return OperationalProfile::MAINTENANCE;
  }
  throw std::invalid_argument("unsupported operational profile: " + value);
}

TfOwnershipValidation validateTfOwnership(
  OperationalProfile profile, const TfOwnerConfig & config)
{
  switch (profile) {
    case OperationalProfile::MAPPING:
      if (config.pgo_enabled && !config.localizer_enabled &&
        config.pgo_publishes_global_tf && !config.localizer_publishes_global_tf)
      {
        return {true, "pgo", ""};
      }
      return {false, "conflict", "mapping requires PGO as the only global TF owner"};
    case OperationalProfile::LOCALIZATION:
      if (!config.pgo_enabled && config.localizer_enabled &&
        !config.pgo_publishes_global_tf && config.localizer_publishes_global_tf)
      {
        return {true, "localizer", ""};
      }
      return {
        false, "conflict", "localization requires localizer as the only global TF owner"};
    case OperationalProfile::MAINTENANCE:
      if (!config.pgo_enabled && !config.localizer_enabled &&
        !config.pgo_publishes_global_tf && !config.localizer_publishes_global_tf)
      {
        return {true, "none", ""};
      }
      return {false, "conflict", "maintenance forbids online global TF publishers"};
  }
  return {false, "conflict", "unknown operational profile"};
}

}  // namespace localizer
