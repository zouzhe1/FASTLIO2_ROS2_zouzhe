#pragma once

#include <string>

namespace localizer
{

enum class OperationalProfile
{
  MAPPING,
  LOCALIZATION,
  MAINTENANCE
};

struct TfOwnerConfig
{
  bool pgo_enabled = false;
  bool localizer_enabled = false;
  bool pgo_publishes_global_tf = false;
  bool localizer_publishes_global_tf = false;
};

struct TfOwnershipValidation
{
  bool valid = false;
  std::string owner = "none";
  std::string reason;
};

OperationalProfile parseOperationalProfile(const std::string & value);
TfOwnershipValidation validateTfOwnership(
  OperationalProfile profile, const TfOwnerConfig & config);

}  // namespace localizer
