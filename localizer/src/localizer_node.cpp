#include <queue>
#include <mutex>
#include <filesystem>
#include <functional>
#include <atomic>
#include <fstream>
#include <thread>
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include "interface/msg/localization_status.hpp"
#include "interface/action/relocalize.hpp"
#include "localizer/localization_state_machine.h"
#include "localizer/tf_policy.h"
#include "localizer/registration_backend.h"
#include "localizer/latest_registration.h"
#include "localizer/registration_quality_gate.h"
#include "localizer/global_recovery.h"
#include "map_tools/map_manifest.h"
#include "map_tools/tile_id.h"
#include "map_tools/transactional_generation.h"
#include "place_recognition/place_index.h"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/fastlio2/body_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    double update_hz = 1.0;
    std::string operational_profile = "localization";
    bool publish_global_tf = true;
    size_t degraded_after_failures = 2;
    size_t lost_after_failures = 5;
    double trusted_timeout_seconds = 5.0;
    size_t recovery_consistent_frames = 3;
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
    builtin_interfaces::msg::Time last_message_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r = M3D::Identity();        // localmap_body_r
    V3D last_t = V3D::Zero();            // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
};

struct RegistrationContext
{
    M3D local_r = M3D::Identity();
    V3D local_t = V3D::Zero();
    builtin_interfaces::msg::Time stamp;
    Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
    bool recovery = false;
};

class LocalizerNode : public rclcpp::Node
{
public:
    using RelocalizeAction = interface::action::Relocalize;
    using RelocalizeGoalHandle = rclcpp_action::ServerGoalHandle<RelocalizeAction>;

    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        loadParameters();
        if (localizer::parseOperationalProfile(m_config.operational_profile) !=
            localizer::OperationalProfile::LOCALIZATION || !m_config.publish_global_tf)
            throw std::runtime_error(
                "localizer_node requires operational_profile=localization and publish_global_tf=true");

        localizer::LocalizationStateMachineConfig health_config;
        health_config.degraded_after_failures = m_config.degraded_after_failures;
        health_config.lost_after_failures = m_config.lost_after_failures;
        health_config.trusted_timeout_seconds = m_config.trusted_timeout_seconds;
        health_config.recovery_consistent_frames = m_config.recovery_consistent_frames;
        m_health = localizer::LocalizationStateMachine(health_config);
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);
        m_registration = std::make_shared<localizer::SmallGicpBackend>(m_registration_config);

        m_reloc_srv = this->create_service<interface::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));
        m_reloc_action = rclcpp_action::create_server<RelocalizeAction>(
            this, "relocalize_action",
            std::bind(&LocalizerNode::handleRelocalizeGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&LocalizerNode::handleRelocalizeCancel, this, std::placeholders::_1),
            std::bind(&LocalizerNode::handleRelocalizeAccepted, this, std::placeholders::_1));

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);
        m_status_pub = this->create_publisher<interface::msg::LocalizationStatus>(
            "localization_status", rclcpp::QoS(1).reliable());

        m_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this));
        m_status_timer = this->create_wall_timer(1s, std::bind(&LocalizerNode::publishStatus, this));
    }

    ~LocalizerNode() override
    {
        m_action_cancel.store(true);
        if (m_action_thread.joinable()) m_action_thread.join();
    }

    rclcpp_action::GoalResponse handleRelocalizeGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const RelocalizeAction::Goal> goal)
    {
        if (m_action_active.load() || !std::filesystem::exists(goal->map_path))
            return rclcpp_action::GoalResponse::REJECT;
        if (goal->mode != RelocalizeAction::Goal::AUTO &&
            goal->mode != RelocalizeAction::Goal::ROUGH_POSE)
            return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleRelocalizeCancel(
        const std::shared_ptr<RelocalizeGoalHandle>)
    {
        m_action_cancel.store(true);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleRelocalizeAccepted(const std::shared_ptr<RelocalizeGoalHandle> goal_handle)
    {
        if (m_action_thread.joinable()) m_action_thread.join();
        m_action_active.store(true);
        m_action_cancel.store(false);
        m_action_thread = std::thread(&LocalizerNode::executeRelocalize, this, goal_handle);
    }

    bool prepareRelocalizationTarget(
        const RelocalizeAction::Goal & goal, Eigen::Isometry3d & initial_guess,
        std::string & message)
    {
        const std::filesystem::path requested(goal.map_path);
        if (std::filesystem::is_regular_file(requested))
        {
            if (goal.mode == RelocalizeAction::Goal::AUTO)
            {
                message = "AUTO requires a tiled generation with places.yaml";
                return false;
            }
            if (!m_localizer->loadMap(requested.string()))
            {
                message = "failed to load compatibility PCD";
                return false;
            }
            std::vector<Eigen::Vector3d> target;
            target.reserve(m_localizer->refineMap()->size());
            for (const auto & point : *m_localizer->refineMap())
                target.emplace_back(point.x, point.y, point.z);
            m_registration->setTarget(target, std::hash<std::string>{}(requested.string()));
        }
        else
        {
            std::filesystem::path generation = requested;
            if (!std::filesystem::is_regular_file(generation / "manifest.yaml"))
            {
                const auto current = map_tools::readCurrentGeneration(requested);
                generation = requested / ("generation-" + std::to_string(current));
            }
            std::ifstream manifest_file(generation / "manifest.yaml");
            std::string manifest_text(
                (std::istreambuf_iterator<char>(manifest_file)), std::istreambuf_iterator<char>());
            const auto manifest = map_tools::parseManifest(manifest_text);
            const auto validation = map_tools::validateManifest(
                manifest, generation, m_config.map_frame, true);
            if (!validation.ok)
            {
                message = validation.reason;
                return false;
            }

            std::string level_id;
            std::vector<std::string> tile_keys;
            if (goal.mode == RelocalizeAction::Goal::AUTO)
            {
                std::ifstream index_file(generation / manifest.keyframe_index);
                std::string index_text(
                    (std::istreambuf_iterator<char>(index_file)), std::istreambuf_iterator<char>());
                auto index = place_recognition::PlaceIndex::deserialize(
                    index_text, {20, 60, 80.0, 3});
                std::vector<place_recognition::Point3> scan;
                {
                    std::lock_guard<std::mutex> lock(m_state.message_mutex);
                    scan.reserve(m_state.last_cloud->size());
                    for (const auto & point : *m_state.last_cloud)
                        scan.push_back({point.x, point.y, point.z});
                }
                const auto candidates = index.query(scan, {});
                if (candidates.empty())
                {
                    message = "place index returned no candidate";
                    return false;
                }
                std::vector<Eigen::Vector3d> source;
                source.reserve(scan.size());
                for (const auto & point : scan) source.emplace_back(point.x, point.y, point.z);
                std::vector<localizer::RecoveryCandidate> evidence;
                std::vector<localizer::RegistrationResult> registrations;
                const auto recovery_started = std::chrono::steady_clock::now();
                for (const auto & candidate : candidates)
                {
                    std::vector<Eigen::Vector3d> candidate_target;
                    candidate_target.reserve(m_registration_config.max_target_points);
                    for (const auto & tile : manifest.tiles)
                    {
                        if (std::find(candidate.metadata.tile_keys.begin(),
                            candidate.metadata.tile_keys.end(), tile.id.stableKey()) ==
                            candidate.metadata.tile_keys.end()) continue;
                        CloudType cloud;
                        if (pcl::io::loadPCDFile<PointType>(
                            (generation / tile.file).string(), cloud) != 0) continue;
                        for (const auto & point : cloud)
                        {
                            if (candidate_target.size() >= m_registration_config.max_target_points) break;
                            candidate_target.emplace_back(point.x, point.y, point.z);
                        }
                    }
                    localizer::RegistrationResult registration;
                    if (!candidate_target.empty())
                    {
                        localizer::SmallGicpBackend verifier(m_registration_config);
                        verifier.setTarget(candidate_target, manifest.generation + candidate.metadata.id + 1);
                        const Eigen::Isometry3d candidate_guess =
                            Eigen::Translation3d(candidate.metadata.x, candidate.metadata.y, candidate.metadata.z) *
                            Eigen::AngleAxisd(candidate.metadata.yaw + candidate.yaw_hint, Eigen::Vector3d::UnitZ());
                        registration = verifier.align(source, candidate_guess);
                    }
                    const bool verified = registration.converged && registration.rmse <= 0.15 &&
                        registration.overlap >= 0.5 && registration.hessian_condition <= 1e5;
                    evidence.push_back({candidate.metadata.id, candidate.metadata.level_id,
                        candidate.score, verified, registration.rmse});
                    registrations.push_back(registration);
                    if (m_action_cancel.load()) break;
                }
                const double elapsed_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - recovery_started).count();
                const auto selected = m_global_recovery.select(
                    evidence, "", elapsed_ms, m_action_cancel.load());
                if (!selected.accepted)
                {
                    message = selected.reason;
                    return false;
                }
                const auto selected_it = std::find_if(candidates.begin(), candidates.end(),
                    [&](const auto & candidate) {return candidate.metadata.id == selected.candidate_id;});
                const auto selected_index = static_cast<std::size_t>(
                    std::distance(candidates.begin(), selected_it));
                level_id = selected_it->metadata.level_id;
                tile_keys = selected_it->metadata.tile_keys;
                initial_guess = registrations[selected_index].transform;
                m_last_candidate_id = selected.candidate_id;
                m_last_ambiguity_margin = selected.ambiguity_margin;
            }
            else
            {
                const auto & pose = goal.initial_pose.pose.pose;
                const double z = pose.position.z;
                for (const auto & level : manifest.levels)
                    if (z >= level.z_min && z <= level.z_max) {level_id = level.id; break;}
                if (level_id.empty())
                {
                    message = "rough pose does not belong to a map level";
                    return false;
                }
                const auto centre = map_tools::tileForPoint(
                    level_id, pose.position.x, pose.position.y);
                for (int dx = -1; dx <= 1; ++dx)
                    for (int dy = -1; dy <= 1; ++dy)
                        tile_keys.push_back(map_tools::TileId{
                            level_id, centre.x + dx, centre.y + dy}.stableKey());
            }

            std::vector<Eigen::Vector3d> target;
            target.reserve(m_registration_config.max_target_points);
            for (const auto & tile : manifest.tiles)
            {
                if (std::find(tile_keys.begin(), tile_keys.end(), tile.id.stableKey()) == tile_keys.end())
                    continue;
                CloudType cloud;
                if (pcl::io::loadPCDFile<PointType>((generation / tile.file).string(), cloud) != 0)
                    continue;
                for (const auto & point : cloud)
                {
                    if (target.size() >= m_registration_config.max_target_points) break;
                    target.emplace_back(point.x, point.y, point.z);
                }
            }
            if (target.empty())
            {
                message = "candidate tiles are missing or empty";
                return false;
            }
            m_registration->setTarget(target, manifest.generation);
            m_active_map_id = manifest.map_id;
            m_active_map_generation = manifest.generation;
            m_active_level_id = level_id;
        }

        if (goal.mode == RelocalizeAction::Goal::ROUGH_POSE)
        {
            const auto & pose = goal.initial_pose.pose.pose;
            initial_guess = Eigen::Isometry3d::Identity();
            initial_guess.translation() = Eigen::Vector3d(
                pose.position.x, pose.position.y, pose.position.z);
            initial_guess.linear() = Eigen::Quaterniond(
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
                .normalized().toRotationMatrix();
        }
        return true;
    }

    void executeRelocalize(const std::shared_ptr<RelocalizeGoalHandle> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<RelocalizeAction::Result>();
        Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
        std::string message;
        try
        {
            if (!prepareRelocalizationTarget(*goal, initial_guess, message))
                throw std::runtime_error(message);
            {
                std::lock_guard<std::recursive_mutex> health_lock(m_health_mutex);
                if (!m_health.beginRelocalization())
                    throw std::runtime_error("localization state rejected recovery request");
            }
            {
                std::lock_guard<std::mutex> lock(m_state.service_mutex);
                m_state.initial_guess = initial_guess.matrix().cast<float>();
                m_state.service_received = true;
                m_state.localize_success = false;
            }

            const auto started = std::chrono::steady_clock::now();
            while (!m_action_cancel.load())
            {
                auto feedback = std::make_shared<RelocalizeAction::Feedback>();
                {
                    std::lock_guard<std::recursive_mutex> health_lock(m_health_mutex);
                    feedback->state = static_cast<uint8_t>(m_health.state());
                }
                feedback->candidates_tested = m_last_candidate_id == 0 ? 0 : 1;
                feedback->best_score = static_cast<float>(m_last_registration.rmse);
                feedback->elapsed_sec = static_cast<float>(
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
                goal_handle->publish_feedback(feedback);
                {
                    std::lock_guard<std::recursive_mutex> health_lock(m_health_mutex);
                    if (m_health.state() == localizer::LocalizationHealth::TRACKING &&
                        m_health.globalPoseValid())
                    {
                        result->success = true;
                        result->message = "relocalization confirmed";
                        result->final_state = static_cast<uint8_t>(m_health.state());
                        ++m_accepted_relocalizations;
                        goal_handle->succeed(result);
                        m_action_active.store(false);
                        return;
                    }
                }
                if (feedback->elapsed_sec > 10.0F) throw std::runtime_error("relocalization deadline exceeded");
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            result->message = "relocalization cancelled";
            {
                std::lock_guard<std::recursive_mutex> health_lock(m_health_mutex);
                m_health.failRelocalization();
                result->final_state = static_cast<uint8_t>(m_health.state());
            }
            ++m_rejected_relocalizations;
            goal_handle->canceled(result);
        }
        catch (const std::exception & error)
        {
            result->success = false;
            result->message = error.what();
            {
                std::lock_guard<std::recursive_mutex> health_lock(m_health_mutex);
                m_health.failRelocalization();
                result->final_state = static_cast<uint8_t>(m_health.state());
            }
            ++m_rejected_relocalizations;
            goal_handle->abort(result);
        }
        m_action_active.store(false);
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        this->declare_parameter("operational_profile", "localization");
        this->declare_parameter("publish_global_tf", true);
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
        m_config.update_hz = config["update_hz"].as<double>();
        this->get_parameter("operational_profile", m_config.operational_profile);
        this->get_parameter("publish_global_tf", m_config.publish_global_tf);
        if (config["degraded_after_failures"])
            m_config.degraded_after_failures = config["degraded_after_failures"].as<size_t>();
        if (config["lost_after_failures"])
            m_config.lost_after_failures = config["lost_after_failures"].as<size_t>();
        if (config["trusted_timeout_seconds"])
            m_config.trusted_timeout_seconds = config["trusted_timeout_seconds"].as<double>();
        if (config["recovery_consistent_frames"])
            m_config.recovery_consistent_frames = config["recovery_consistent_frames"].as<size_t>();

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();
        if (config["registration_voxel_resolution"])
            m_registration_config.voxel_resolution = config["registration_voxel_resolution"].as<double>();
        if (config["registration_max_source_points"])
            m_registration_config.max_source_points = config["registration_max_source_points"].as<size_t>();
        if (config["registration_max_target_points"])
            m_registration_config.max_target_points = config["registration_max_target_points"].as<size_t>();
        if (config["registration_max_iterations"])
            m_registration_config.max_iterations = config["registration_max_iterations"].as<int>();
        if (config["registration_max_correspondence_distance"])
            m_registration_config.max_correspondence_distance = config["registration_max_correspondence_distance"].as<double>();
        if (config["registration_threads"])
            m_registration_config.num_threads = config["registration_threads"].as<int>();
        if (config["registration_deadline_ms"])
            m_registration_config.deadline_ms = config["registration_deadline_ms"].as<double>();
    }
    void timerCB()
    {
        std::lock_guard<std::recursive_mutex> health_lock(m_health_mutex);
        const double now_seconds = this->now().seconds();
        m_health.tick(now_seconds);
        if (!m_health.shouldPublishGlobalTf())
            m_last_tf_published = false;

        if (auto completed = m_registration_worker.takeLatest())
        {
            const auto & registration = completed->result;
            const Eigen::Isometry3d correction =
                m_latest_context.initial_guess.inverse() * registration.transform;
            localizer::QualityInput quality;
            quality.converged = registration.converged;
            quality.rmse = registration.rmse;
            quality.inliers = registration.inliers;
            quality.inlier_ratio = registration.inlier_ratio;
            quality.overlap = registration.overlap;
            quality.hessian_condition = registration.hessian_condition;
            quality.correction_translation = correction.translation().norm();
            quality.correction_rotation_deg =
                Eigen::AngleAxisd(correction.rotation()).angle() * 57.29577951308232;
            quality.elapsed_ms = registration.elapsed_ms;
            quality.tiles_complete = true;
            quality.ambiguity_margin = m_latest_context.recovery ? m_last_ambiguity_margin : 0.0;
            const auto decision = m_quality_gate.evaluate(
                quality, m_latest_context.recovery ?
                localizer::QualityMode::RECOVERY : localizer::QualityMode::TRACKING);
            m_last_registration = registration;
            m_last_quality_reason = decision.reason;

            if (decision.accepted)
            {
                const M3D map_body_r = registration.transform.rotation();
                const V3D map_body_t = registration.transform.translation();
                m_state.last_offset_r = map_body_r * m_latest_context.local_r.transpose();
                m_state.last_offset_t =
                    -map_body_r * m_latest_context.local_r.transpose() * m_latest_context.local_t +
                    map_body_t;
                if (m_health.state() == localizer::LocalizationHealth::RELOCALIZING)
                    m_health.acceptRecoveryCandidate();
                if (m_health.state() == localizer::LocalizationHealth::RECOVERING)
                {
                    if (m_health.confirmRecoveryFrame(true, now_seconds))
                    {
                        std::lock_guard<std::mutex> lock(m_state.service_mutex);
                        m_state.localize_success = true;
                        m_state.service_received = false;
                    }
                }
                else
                    m_health.acceptTrackingResult(now_seconds);
            }
            else if (m_latest_context.recovery)
            {
                m_health.failRelocalization();
                std::lock_guard<std::mutex> lock(m_state.service_mutex);
                m_state.localize_success = false;
                m_state.service_received = false;
            }
            else
                m_health.rejectTrackingResult(now_seconds);

            if (m_config.publish_global_tf && m_health.shouldPublishGlobalTf())
            {
                sendBroadCastTF(m_latest_context.stamp);
                m_last_tf_published = true;
            }
            else
                m_last_tf_published = false;
            publishMapCloud(m_latest_context.stamp);
        }

        if (!m_state.message_received)
            return;

        rclcpp::Duration diff = this->now() - m_state.last_send_tf_time;
        if (diff.seconds() <= (1.0 / m_config.update_hz))
            return;

        m_state.last_send_tf_time = this->now();

        M4F initial_guess = M4F::Identity();
        if (m_state.service_received)
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
            // m_state.service_received = false;
        }
        else
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
        }

        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        std::vector<Eigen::Vector3d> source_points;
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            source_points.reserve(m_state.last_cloud->size());
            for (const auto & point : *m_state.last_cloud)
                source_points.emplace_back(point.x, point.y, point.z);
        }

        Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
        guess.matrix() = initial_guess.cast<double>();
        m_latest_context = {
            current_local_r, current_local_t, current_time, guess,
            m_health.state() == localizer::LocalizationHealth::RELOCALIZING ||
            m_health.state() == localizer::LocalizationHealth::RECOVERING};
        const std::uint64_t request_id = ++m_registration_sequence;
        const auto backend = m_registration;
        m_registration_worker.submit({
            request_id,
            [backend, points = std::move(source_points), guess]() {
                return backend->align(points, guess);
            }});
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex>(m_state.message_mutex);

        pcl::fromROSMsg(*cloud_msg, *m_state.last_cloud);

        m_state.last_r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                            odom_msg->pose.pose.orientation.x,
                                            odom_msg->pose.pose.orientation.y,
                                            odom_msg->pose.pose.orientation.z)
                             .toRotationMatrix();
        m_state.last_t = V3D(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
        m_state.last_message_time = cloud_msg->header.stamp;
        if (!m_state.message_received)
        {
            m_state.message_received = true;
            m_config.local_frame = odom_msg->header.frame_id;
        }
    }

    void sendBroadCastTF(const builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, std::shared_ptr<interface::srv::Relocalize::Response> response)
    {
        std::lock_guard<std::recursive_mutex> health_lock(m_health_mutex);
        std::string pcd_path = request->pcd_path;
        float x = request->x;
        float y = request->y;
        float z = request->z;
        float yaw = request->yaw;
        float roll = request->roll;
        float pitch = request->pitch;

        if (!std::filesystem::exists(pcd_path))
        {
            response->success = false;
            response->message = "pcd file not found";
            return;
        }

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        bool load_flag = m_localizer->loadMap(pcd_path);
        if (!load_flag)
        {
            response->success = false;
            response->message = "load map failed";
            return;
        }
        if (!m_health.beginRelocalization())
        {
            response->success = false;
            response->message = "localizer is not ready to start relocalization";
            return;
        }
        std::vector<Eigen::Vector3d> target_points;
        target_points.reserve(m_localizer->refineMap()->size());
        for (const auto & point : *m_localizer->refineMap())
            target_points.emplace_back(point.x, point.y, point.z);
        m_registration->setTarget(target_points, std::hash<std::string>{}(pcd_path));

        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
            m_state.localize_success = false;
        }

        response->success = true;
        response->message = "relocalization request accepted";
        return;
    }

    void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        std::lock_guard<std::recursive_mutex> health_lock(m_health_mutex);
        (void)request;
        std::lock_guard<std::mutex> lock(m_state.service_mutex);
        response->valid = m_state.localize_success && m_health.globalPoseValid();
        return;
    }

    void publishStatus()
    {
        std::lock_guard<std::recursive_mutex> health_lock(m_health_mutex);
        const double now_seconds = this->now().seconds();
        m_health.tick(now_seconds);
        if (!m_health.shouldPublishGlobalTf())
            m_last_tf_published = false;

        interface::msg::LocalizationStatus status;
        status.header.stamp = this->now();
        status.header.frame_id = m_config.map_frame;
        status.state = static_cast<uint8_t>(m_health.state());
        status.global_pose_valid = m_health.globalPoseValid();
        status.global_tf_published = m_last_tf_published;
        status.operational_profile = m_config.operational_profile;
        status.tf_owner = "localizer";
        status.reason = healthName(m_health.state());
        if (!m_last_quality_reason.empty())
            status.reason += ":" + m_last_quality_reason;
        status.rmse = static_cast<float>(m_last_registration.rmse);
        status.inliers = static_cast<uint32_t>(m_last_registration.inliers);
        status.inlier_ratio = static_cast<float>(m_last_registration.inlier_ratio);
        status.overlap = static_cast<float>(m_last_registration.overlap);
        status.hessian_condition = static_cast<float>(m_last_registration.hessian_condition);
        status.registration_ms = static_cast<float>(m_last_registration.elapsed_ms);
        status.replaced_work_items = m_registration_worker.replacedCount();
        status.map_id = m_active_map_id;
        status.map_generation = m_active_map_generation;
        status.level_id = m_active_level_id;
        status.candidate_id = m_last_candidate_id;
        status.ambiguity_margin = static_cast<float>(m_last_ambiguity_margin);
        status.accepted_relocalization_count = m_accepted_relocalizations.load();
        status.rejected_relocalization_count = m_rejected_relocalizations.load();
        if (m_health.lastTrustedTime() >= 0.0)
        {
            const int64_t trusted_nanoseconds = static_cast<int64_t>(
                m_health.lastTrustedTime() * 1e9);
            status.last_trusted_update.sec = static_cast<int32_t>(
                trusted_nanoseconds / 1000000000LL);
            status.last_trusted_update.nanosec = static_cast<uint32_t>(
                trusted_nanoseconds % 1000000000LL);
            status.correction_age_sec = static_cast<float>(
                std::max(0.0, now_seconds - m_health.lastTrustedTime()));
        }
        m_status_pub->publish(status);
    }

    static std::string healthName(localizer::LocalizationHealth health)
    {
        switch (health)
        {
        case localizer::LocalizationHealth::UNINITIALIZED:
            return "uninitialized";
        case localizer::LocalizationHealth::TRACKING:
            return "tracking";
        case localizer::LocalizationHealth::DEGRADED:
            return "degraded";
        case localizer::LocalizationHealth::LOST:
            return "lost";
        case localizer::LocalizationHealth::RELOCALIZING:
            return "relocalizing";
        case localizer::LocalizationHealth::RECOVERING:
            return "recovering";
        }
        return "unknown";
    }
    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        if (m_map_cloud_pub->get_subscription_count() < 1)
            return;
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (map_cloud->size() < 1)
            return;
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

private:
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    localizer::RegistrationConfig m_registration_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    std::shared_ptr<localizer::SmallGicpBackend> m_registration;
    localizer::LatestRegistration m_registration_worker;
    localizer::RegistrationQualityGate m_quality_gate{{}};
    localizer::GlobalRecovery m_global_recovery{{3, 3000.0, 0.15, 3}};
    RegistrationContext m_latest_context;
    localizer::RegistrationResult m_last_registration;
    std::string m_last_quality_reason;
    std::uint64_t m_registration_sequence = 0;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::TimerBase::SharedPtr m_status_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    rclcpp::Publisher<interface::msg::LocalizationStatus>::SharedPtr m_status_pub;
    localizer::LocalizationStateMachine m_health;
    std::recursive_mutex m_health_mutex;
    bool m_last_tf_published = false;
    rclcpp_action::Server<RelocalizeAction>::SharedPtr m_reloc_action;
    std::thread m_action_thread;
    std::atomic<bool> m_action_active{false};
    std::atomic<bool> m_action_cancel{false};
    std::string m_active_map_id;
    std::string m_active_level_id;
    std::uint64_t m_active_map_generation = 0;
    std::uint64_t m_last_candidate_id = 0;
    double m_last_ambiguity_margin = 1.0;
    std::atomic<std::uint32_t> m_accepted_relocalizations{0};
    std::atomic<std::uint32_t> m_rejected_relocalizations{0};
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
