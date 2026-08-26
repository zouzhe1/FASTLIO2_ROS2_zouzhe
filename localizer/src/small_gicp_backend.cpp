#include "localizer/registration_backend.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <small_gicp/registration/registration_helper.hpp>

namespace localizer
{

struct SmallGicpBackend::Impl
{
  explicit Impl(RegistrationConfig value) : config(std::move(value)) {}
  RegistrationConfig config;
  small_gicp::PointCloud::Ptr target;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> target_tree;
  std::uint64_t target_generation{0};
  std::uint64_t preprocess_count{0};
};

namespace
{
std::vector<Eigen::Vector4d> boundedPoints(
  const std::vector<Eigen::Vector3d> & input, std::size_t limit)
{
  const std::size_t count = std::min(input.size(), limit);
  std::vector<Eigen::Vector4d> output;
  output.reserve(count);
  if (count == 0) return output;
  const double stride = static_cast<double>(input.size()) / static_cast<double>(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto & point = input[std::min(
      static_cast<std::size_t>(i * stride), input.size() - 1)];
    if (point.allFinite()) output.emplace_back(point.x(), point.y(), point.z(), 1.0);
  }
  return output;
}
}  // namespace

SmallGicpBackend::SmallGicpBackend(RegistrationConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
  if (impl_->config.voxel_resolution <= 0.0 || impl_->config.max_source_points == 0 ||
    impl_->config.max_iterations <= 0 || impl_->config.num_threads <= 0) {
    throw std::invalid_argument("small_gicp bounds must be positive");
  }
}

SmallGicpBackend::~SmallGicpBackend() = default;

void SmallGicpBackend::setTarget(
  const std::vector<Eigen::Vector3d> & points, std::uint64_t generation)
{
  if (impl_->target && generation == impl_->target_generation) return;
  const auto bounded = boundedPoints(points, impl_->config.max_target_points);
  if (bounded.empty()) throw std::invalid_argument("registration target is empty");
  std::tie(impl_->target, impl_->target_tree) = small_gicp::preprocess_points(
    bounded, impl_->config.voxel_resolution, 10, impl_->config.num_threads);
  impl_->target_generation = generation;
  ++impl_->preprocess_count;
}

RegistrationResult SmallGicpBackend::align(
  const std::vector<Eigen::Vector3d> & source,
  const Eigen::Isometry3d & initial_guess) const
{
  RegistrationResult output;
  output.target_generation = impl_->target_generation;
  if (!impl_->target || !impl_->target_tree) {
    output.reason = "target_not_ready";
    return output;
  }
  if (source.empty()) {
    output.reason = "empty_source";
    return output;
  }
  if (impl_->config.deadline_ms <= 0.0) {
    output.reason = "deadline_exceeded";
    return output;
  }
  const auto started = std::chrono::steady_clock::now();
  const auto points = boundedPoints(source, impl_->config.max_source_points);
  auto source_cloud = small_gicp::preprocess_points(
    points, impl_->config.voxel_resolution, 10, impl_->config.num_threads).first;
  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.max_correspondence_distance = impl_->config.max_correspondence_distance;
  setting.max_iterations = impl_->config.max_iterations;
  setting.num_threads = impl_->config.num_threads;
  const auto result = small_gicp::align(
    *impl_->target, *source_cloud, *impl_->target_tree, initial_guess, setting);
  output.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  output.transform = result.T_target_source;
  output.inliers = result.num_inliers;
  output.rmse = result.num_inliers > 0 ? std::sqrt(
    std::max(0.0, result.error) / static_cast<double>(result.num_inliers)) :
    std::numeric_limits<double>::infinity();
  output.inlier_ratio = source_cloud->empty() ? 0.0 :
    static_cast<double>(result.num_inliers) / static_cast<double>(source_cloud->size());
  output.overlap = std::min(1.0, static_cast<double>(result.num_inliers) /
    static_cast<double>(std::max<std::size_t>(1, std::min(source_cloud->size(), impl_->target->size()))));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(result.H);
  if (eigen.info() == Eigen::Success && eigen.eigenvalues().minCoeff() > 1e-12) {
    output.hessian_condition = eigen.eigenvalues().maxCoeff() / eigen.eigenvalues().minCoeff();
  } else {
    output.hessian_condition = std::numeric_limits<double>::infinity();
  }
  if (output.elapsed_ms > impl_->config.deadline_ms) {
    output.reason = "deadline_exceeded";
    return output;
  }
  if (!result.converged) {
    output.reason = "not_converged";
    return output;
  }
  if (!output.transform.matrix().allFinite()) {
    output.reason = "non_finite_transform";
    return output;
  }
  output.converged = true;
  output.reason = "ok";
  return output;
}

std::uint64_t SmallGicpBackend::targetPreprocessCount() const
{
  return impl_->preprocess_count;
}

}  // namespace localizer
