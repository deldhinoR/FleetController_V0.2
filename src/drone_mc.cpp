#include <chrono>
#include <array>
#include <string>
#include <vector>
#include <cmath>
#include <regex>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class DroneMC : public rclcpp::Node {
public:
    DroneMC() : Node("drone_node"), leader_id_(1) {
        // Declare parameters
        this->declare_parameter<int>("drone_id", 0);
        this->get_parameter("drone_id", drone_id_);

        this->declare_parameter<std::string>("formation_mode", "format_2");
        this->get_parameter("formation_mode", formation_mode_);

        // Optional publisher for leader ID
        leader_pub_ = this->create_publisher<std_msgs::msg::Int32>("/fleet/leader_id", 10);

        // Compute initial formation offsets
        compute_formation_offset();

        // QoS setup
        rclcpp::QoS mavros_qos(50);
        mavros_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        mavros_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // Subscribe to leader ID
        leader_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/fleet/leader_id", mavros_qos,
            [this](std_msgs::msg::Int32::SharedPtr msg){
                leader_id_ = msg->data;
                is_leader_ = (drone_id_ == leader_id_);
            });
        
        rclcpp::QoS qos_formation(10);
        formation_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/fleet/formation_mode", qos_formation,
            [this](const std_msgs::msg::String::SharedPtr msg) {  
                formation_mode_ = msg->data;
                compute_formation_offset();
                RCLCPP_INFO(this->get_logger(),
                            "Formation mode changed to %s, recomputed offset",
                    formation_mode_.c_str());
    });


        // Status timer
        status_timer_ = this->create_wall_timer(
            10s, [this]() {
                RCLCPP_INFO(this->get_logger(), "I am %s (drone%d), leader=%d, formation=%s",
                            is_leader_ ? "LEADER" : "FOLLOWER", drone_id_, leader_id_, formation_mode_.c_str());
            });

        // Publisher for drone setpoint
        std::string topic = "/drone" + std::to_string(drone_id_) + "/setpoint_position/local";
        setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic, mavros_qos);

        // Subscribe to leader setpoint
        leader_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/fleet/leader_setpoint", mavros_qos,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                geometry_msgs::msg::PoseStamped target;
                target.header.stamp = this->now();
                target.header.frame_id = "map";

                if (is_leader_) {
                    target.pose = msg->pose;  // leader just follows its own setpoint
                } else {
                    // Leader position
                    Eigen::Vector3d leader_pos(msg->pose.position.x,
                                               msg->pose.position.y,
                                               msg->pose.position.z);

                    // Leader orientation (quaternion)
                    Eigen::Quaterniond q(msg->pose.orientation.w,
                                         msg->pose.orientation.x,
                                         msg->pose.orientation.y,
                                         msg->pose.orientation.z);

                    // Extract yaw rotation around Z
                    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
                    double yaw = euler[2];

                    // Rotation matrix for yaw
                    Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

                    // Apply rotation to formation offset
                    Eigen::Vector3d offset_vec(offset_[0], offset_[1], offset_[2]);
                    Eigen::Vector3d rotated_offset = R * offset_vec;

                    // Compute drone target position
                    Eigen::Vector3d target_pos = leader_pos + rotated_offset;
                    target.pose.position.x = target_pos.x();
                    target.pose.position.y = target_pos.y();
                    target.pose.position.z = target_pos.z();

                    // Copy leader orientation
                    target.pose.orientation = msg->pose.orientation;
                }

                // Publish target setpoint
                setpoint_pub_->publish(target);
            });

        is_leader_ = (drone_id_ == leader_id_);
    }

private:
    void compute_formation_offset() {
        // Default side length
        double triangle_side = 2.0;

        // Parse formation_mode like "format_5"
        std::smatch match;
        std::regex re("format_(\\d+)");
        if (std::regex_match(formation_mode_, match, re)) {
            triangle_side = std::stod(match[1].str());
        }

        double height = std::sqrt(3) / 2 * triangle_side;

        if(leader_id_ == 1){
        switch(drone_id_) {
            case 1: // leader
                offset_ = {0.0, 0.0, 0.0};
                break;
            case 2:
                offset_ = {-height, -triangle_side / 2.0, 0.0};
                break;
            case 3:
                offset_ = {height, -triangle_side * 2, 0.0};
                break;
            default:
                offset_ = {0.0, 0.0, 0.0};
                break;
                }
            }
        
        // if(leader_id_ == 2){
        // switch(drone_id_) {
        //     case 1:
        //         offset_ = {height, triangle_side/2.0, 0.0 };
        //         break;
        //     case 2: // leader
        //         offset_ = {0.0, 0.0, 0.0};
        //         break;
        //     case 3:
        //         offset_ = {2*height, -3*triangle_side/2.0, 0.0} ;
        //         break;
        //     default:
        //         offset_ = {0.0, 0.0, 0.0};
        //         break;
        //         }
        //     }

        // if(leader_id_ == 3){
        // switch(drone_id_) {
        //     case 1: 
        //         offset_ = { -height,   triangle_side*2.0, 0.0 };
        //         break;
        //     case 2:
        //         offset_ = {-2*height,  3*triangle_side/2.0, 0.0 };
        //         break;
        //     case 3:// leader
        //         offset_ = {0.0, 0.0, 0.0};
        //         break;
        //     default:
        //         offset_ = {0.0, 0.0, 0.0};
        //         break;
        //             }
        //         }
        //     }

    int drone_id_;
    int leader_id_;
    bool is_leader_;
    std::string formation_mode_;
    std::array<double,3> offset_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr leader_id_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr formation_sub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leader_setpoint_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr leader_pub_;
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
