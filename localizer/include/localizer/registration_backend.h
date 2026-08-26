#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace localizer
{

struct RegistrationConfig
{
  double voxel_resolution{0.20};
  std::size_t max_source_points{12000};
  std::size_t max_target_points{120000};
  int max_iterations{20};
  double max_correspondence_distance{1.0};
  int num_threads{2};
  double deadline_ms{80.0};
};

struct RegistrationResult
{
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
  bool converged{false};
  double rmse{0.0};
  std::size_t inliers{0};
  double inlier_ratio{0.0};
  double overlap{0.0};
  double hessian_condition{0.0};
  double elapsed_ms{0.0};
  std::string reason;
  std::uint64_t target_generation{0};
};

class RegistrationBackend
{
public:
  virtual ~RegistrationBackend() = default;
  virtual void setTarget(
    const std::vector<Eigen::Vector3d> & points, std::uint64_t generation) = 0;
  virtual RegistrationResult align(
    const std::vector<Eigen::Vector3d> & source,
    const Eigen::Isometry3d & initial_guess) const = 0;
};

class SmallGicpBackend : public RegistrationBackend
{
public:
  explicit SmallGicpBackend(RegistrationConfig config);
  ~SmallGicpBackend() override;
  void setTarget(
    const std::vector<Eigen::Vector3d> & points, std::uint64_t generation) override;
  RegistrationResult align(
    const std::vector<Eigen::Vector3d> & source,
    const Eigen::Isometry3d & initial_guess) const override;
  std::uint64_t targetPreprocessCount() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace localizer
