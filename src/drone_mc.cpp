#include <chrono>
#include <array>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class DroneMC : public rclcpp::Node {
public:
    DroneMC() : Node("drone_node"), leader_id_(1) {
        this->declare_parameter<int>("drone_id", 0);
        this->get_parameter("drone_id", drone_id_);

        std::vector<double> offset_vec = this->declare_parameter<std::vector<double>>("offset", {0.0, 0.0, 0.0});
        offset_ = {offset_vec[0], offset_vec[1], offset_vec[2]};

        // -------------------- QoS Setup --------------------
        rclcpp::QoS mavros_qos(50);  // increased queue depth
        mavros_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        mavros_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        // ----------------------------------------------------

        // Subscribe to leader ID using improved QoS
        leader_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/fleet/leader_id", mavros_qos, 
            [this](std_msgs::msg::Int32::SharedPtr msg){
                leader_id_ = msg->data;
                is_leader_ = (drone_id_ == leader_id_);
                subscribe_to_leader_pose();
            });

        // Status timer
        status_timer_ = this->create_wall_timer(
            10s, [this]() {
                RCLCPP_INFO(this->get_logger(), "I am %s (drone%d), leader=%d",
                            is_leader_ ? "LEADER" : "FOLLOWER", drone_id_, leader_id_);
            });

        // Publisher for local setpoint using improved QoS
        std::string topic = "/drone" + std::to_string(drone_id_) + "/setpoint_position/local";
        setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic, mavros_qos);

        // Subscribe to leader setpoints using improved QoS
        leader_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/fleet/leader_setpoint", mavros_qos,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                geometry_msgs::msg::PoseStamped target;
                target.header.stamp = this->now();
                target.header.frame_id = "map";
                target.pose.position.x = msg->pose.position.x + offset_[0];
                target.pose.position.y = msg->pose.position.y + offset_[1];
                target.pose.position.z = msg->pose.position.z + offset_[2];
                target.pose.orientation = msg->pose.orientation;
                setpoint_pub_->publish(target);
            });

        is_leader_ = (drone_id_ == leader_id_);
    }

private:
    void subscribe_to_leader_pose() {
        is_leader_ = (drone_id_ == leader_id_);
    }

    int drone_id_;
    int leader_id_;
    bool is_leader_;
    std::array<double,3> offset_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr leader_id_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leader_setpoint_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneMC>();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
