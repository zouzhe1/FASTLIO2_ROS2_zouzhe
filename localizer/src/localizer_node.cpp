#include <queue>
#include <mutex>
#include <filesystem>
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
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
#include "localizer/localization_state_machine.h"
#include "localizer/tf_policy.h"
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

class LocalizerNode : public rclcpp::Node
{
public:
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

        m_reloc_srv = this->create_service<interface::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);
        m_status_pub = this->create_publisher<interface::msg::LocalizationStatus>(
            "localization_status", rclcpp::QoS(1).reliable());

        m_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this));
        m_status_timer = this->create_wall_timer(1s, std::bind(&LocalizerNode::publishStatus, this));
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
    }
    void timerCB()
    {
        const double now_seconds = this->now().seconds();
        m_health.tick(now_seconds);
        if (!m_health.shouldPublishGlobalTf())
            m_last_tf_published = false;
        if (!m_state.message_received)
            return;

        rclcpp::Duration diff = this->now() - m_state.last_send_tf_time;

        bool update_tf = diff.seconds() > (1.0 / m_config.update_hz) && m_state.message_received;

        if (!update_tf)
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
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            m_localizer->setInput(m_state.last_cloud);
        }

        bool result = m_localizer->align(initial_guess);
        if (result)
        {
            M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            m_state.last_offset_r = map_body_r * current_local_r.transpose();
            m_state.last_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;
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
        else if (m_health.state() == localizer::LocalizationHealth::RELOCALIZING ||
                 m_health.state() == localizer::LocalizationHealth::RECOVERING)
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
            sendBroadCastTF(current_time);
            m_last_tf_published = true;
        }
        else
            m_last_tf_published = false;
        publishMapCloud(current_time);
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
        (void)request;
        std::lock_guard<std::mutex> lock(m_state.service_mutex);
        response->valid = m_state.localize_success && m_health.globalPoseValid();
        return;
    }

    void publishStatus()
    {
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
    std::shared_ptr<ICPLocalizer> m_localizer;
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
    bool m_last_tf_published = false;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
