#include <chrono>
#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <mutex>
#include <algorithm>
#include <cctype>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

class GroundControl : public rclcpp::Node {
public:
    GroundControl() : Node("ground_controller"), leader_id_(1) {
        setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/fleet/leader_setpoint", 10);

        leader_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/fleet/leader_id", 10);

        formation_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/fleet/formation_mode", 10);

        timer_ = this->create_wall_timer(
            50ms, std::bind(&GroundControl::publish_setpoint, this));

        x_ = 0.0;
        y_ = 0.0;
        z_ = 10.0;

        for (const auto &drone_ns : {"drone1", "drone2", "drone3"}) {
            setmode_clients_[drone_ns] = this->create_client<mavros_msgs::srv::SetMode>(
                std::string(drone_ns) + "/set_mode");
            arm_clients_[drone_ns] = this->create_client<mavros_msgs::srv::CommandBool>(
                std::string(drone_ns) + "/cmd/arming");
        }

        RCLCPP_INFO(this->get_logger(), "GroundControl started.");
    }

    void run_input_loop() {
        std::string line;
        while (rclcpp::ok()) {
            try {
                if (!std::getline(std::cin, line)) break;

                trim(line);
                std::transform(line.begin(), line.end(), line.begin(), ::tolower);

                if (line.empty()) continue;

                if (line == "land") { publish_land(); continue; }
                if (line == "ready") { publish_offboard(); continue; }
                if (line == "arm") { publish_arm(); continue; }
                if (line.rfind("format_", 0) == 0) {
                    std_msgs::msg::String msg;
                    msg.data = line;  // e.g., "format_5"
                    formation_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "Formation updated to: %s", line.c_str());
                    continue;
                }
                if (line == "manual") {
                    manual_mode = true;
                    manual_control_loop();
                    continue;
                }

                if (line == "land_1") { publish_land_only(1); continue; }
                if (line == "ready_1") { publish_offboard_only(1); continue; }
                if (line == "arm_1") { publish_arm_only(1); continue; }
                if (line == "land_2") { publish_land_only(2); continue; }
                if (line == "ready_2") { publish_offboard_only(2); continue; }
                if (line == "arm_2") { publish_arm_only(2); continue; }
                if (line == "land_3") { publish_land_only(3); continue; }
                if (line == "ready_3") { publish_offboard_only(3); continue; }
                if (line == "arm_3") { publish_arm_only(3); continue; }

                if (line == "exit") {
                    RCLCPP_INFO(this->get_logger(), "Exit command received. Shutting down...");
                    rclcpp::shutdown();
                    break;
                }

                // leader change command
                if (line.rfind("leader", 0) == 0) {
                    std::string arg = line.substr(6);
                    trim(arg);
                    try {
                        int new_leader = std::stoi(arg);
                        leader_id_ = new_leader;
                        std_msgs::msg::Int32 msg;
                        msg.data = leader_id_;
                        leader_pub_->publish(msg);
                        RCLCPP_INFO(this->get_logger(), "New leader: drone%d", leader_id_);
                    } catch (...) {
                        RCLCPP_WARN(this->get_logger(), "Invalid leader command. Example: leader 2");
                    }
                    continue;
                }

                // position command
                double nx, ny, nz;
                if (parse_line(line, nx, ny, nz)) {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    x_ = nx;
                    y_ = ny;
                    z_ = nz;
                    RCLCPP_INFO(this->get_logger(), "Leader target updated: %.2f, %.2f, %.2f", x_, y_, z_);
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "Invalid input. Use x:y:z, 'leader <id>', or commands 'land', 'arm', 'ready', 'exit'");
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in input loop: %s", e.what());
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Unknown exception in input loop");
            }
        }
    }

private:
    static inline void trim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            [](unsigned char ch){ return !std::isspace(ch); }));
        s.erase(std::find_if(s.rbegin(), s.rend(),
            [](unsigned char ch){ return !std::isspace(ch); }).base(), s.end());
    }

    bool parse_line(const std::string &line, double &nx, double &ny, double &nz) {
        std::stringstream ss(line);
        std::string token;
        try {
            if (!std::getline(ss, token, ':')) return false;
            nx = std::stod(token);

            if (!std::getline(ss, token, ':')) return false;
            ny = std::stod(token);

            if (!std::getline(ss, token, ':')) return false;
            nz = std::stod(token);
        } catch (...) {
            return false;
        }
        return true;
    }

    char get_keyPress() {
    char buf = 0;
    struct termios old = {};
    if (tcgetattr(STDIN_FILENO, &old) < 0) perror("tcgetattr()");
    struct termios newt = old;
    newt.c_lflag &= ~(ICANON | ECHO); // disable buffering and echo
    if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) < 0) perror("tcsetattr ICANON");

    if (read(STDIN_FILENO, &buf, 1) < 0) perror("read()");

    if (buf == '\033') { // escape sequence
        char seq[2];
        if (read(STDIN_FILENO, &seq[0], 1) == 0) { tcsetattr(STDIN_FILENO, TCSANOW, &old); return '\033'; } // ESC key alone
        if (read(STDIN_FILENO, &seq[1], 1) == 0) { tcsetattr(STDIN_FILENO, TCSANOW, &old); return '\033'; }

        // Arrow keys send: ESC [ A/B/C/D
        if (seq[0] == '[') {
            switch(seq[1]) {
                case 'A': buf = 'U'; break; // UP
                case 'B': buf = 'D'; break; // DOWN
                case 'C': buf = 'R'; break; // RIGHT
                case 'D': buf = 'L'; break; // LEFT
                default: buf = 0; break;
            }
        }
    }

    if (tcsetattr(STDIN_FILENO, TCSANOW, &old) < 0) perror("tcsetattr ~ICANON");
    return buf;
}


    void publish_setpoint() {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        geometry_msgs::msg::PoseStamped sp;
        sp.header.stamp = this->now();
        sp.header.frame_id = "map";
        sp.pose.position.x = x_;
        sp.pose.position.y = y_;
        sp.pose.position.z = z_;
        sp.pose.orientation.w = 1.0;

        setpoint_pub_->publish(sp);

        std_msgs::msg::Int32 msg;
        msg.data = leader_id_;
        leader_pub_->publish(msg);
    }

    // ---------------- MAVROS functions ----------------
    void publish_land() {
        for (const auto &drone_ns : {"drone1", "drone2", "drone3"}) {
            auto client = setmode_clients_[drone_ns];
            if (!client->wait_for_service(5s)) {
                RCLCPP_ERROR(this->get_logger(), "%s SetMode service unavailable", drone_ns);
                continue;
            }

            auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            request->custom_mode = "AUTO.LAND";

            client->async_send_request(request, [this, drone_ns](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                auto response = future.get();
                if (response->mode_sent)
                    RCLCPP_INFO(this->get_logger(), "%s LAND mode sent successfully", drone_ns);
                else
                    RCLCPP_WARN(this->get_logger(), "%s LAND mode failed", drone_ns);
            });
        }
    }

    void publish_offboard() {
        for (const auto &drone_ns : {"drone1", "drone2", "drone3"}) {
            auto client = setmode_clients_[drone_ns];
            if (!client->wait_for_service(5s)) {
                RCLCPP_ERROR(this->get_logger(), "%s SetMode service unavailable", drone_ns);
                continue;
            }

            auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            request->custom_mode = "OFFBOARD";

            client->async_send_request(request, [this, drone_ns](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                auto response = future.get();
                if (response->mode_sent)
                    RCLCPP_INFO(this->get_logger(), "%s OFFBOARD mode sent successfully", drone_ns);
                else
                    RCLCPP_WARN(this->get_logger(), "%s OFFBOARD mode failed", drone_ns);
            });
        }
    }

    void publish_arm() {
        for (const auto &drone_ns : {"drone1", "drone2", "drone3"}) {
            auto client = arm_clients_[drone_ns];
            if (!client->wait_for_service(5s)) {
                RCLCPP_ERROR(this->get_logger(), "%s Arming service unavailable", drone_ns);
                continue;
            }

            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = true;

            client->async_send_request(request, [this, drone_ns](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                auto response = future.get();
                if (response->success)
                    RCLCPP_INFO(this->get_logger(), "%s Vehicle ARMED successfully", drone_ns);
                else
                    RCLCPP_WARN(this->get_logger(), "%s ARM failed", drone_ns);
            });
        }
    }

    void publish_land_only(int drone_no) {
        std::string drone_ns = "drone" + std::to_string(drone_no);
        auto client = setmode_clients_[drone_ns];

        if (!client) {
            RCLCPP_ERROR(this->get_logger(), "%s SetMode client not available", drone_ns.c_str());
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "AUTO.LAND";

        client->async_send_request(request,
            [this, drone_ns](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture /*future*/) {
                RCLCPP_INFO(this->get_logger(), "%s LAND request sent", drone_ns.c_str());
            });
    }

    void publish_offboard_only(int drone_no) {
        std::string drone_ns = "drone" + std::to_string(drone_no);
        auto client = setmode_clients_[drone_ns];

        if (!client) {
            RCLCPP_ERROR(this->get_logger(), "%s SetMode client not available", drone_ns.c_str());
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";

        client->async_send_request(request,
            [this, drone_ns](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture /*future*/) {
                RCLCPP_INFO(this->get_logger(), "%s OFFBOARD request sent", drone_ns.c_str());
            });
    }

    void publish_arm_only(int drone_no) {
        std::string drone_ns = "drone" + std::to_string(drone_no);
        auto client = arm_clients_[drone_ns];

        if (!client) {
            RCLCPP_ERROR(this->get_logger(), "%s Arming client not available", drone_ns.c_str());
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        client->async_send_request(request,
            [this, drone_ns](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture /*future*/) {
                RCLCPP_INFO(this->get_logger(), "%s ARM request sent", drone_ns.c_str());
            });
    }

    void manual_control_loop() {
    RCLCPP_INFO(this->get_logger(), "Manual mode activated. Arrow keys to move, ESC to exit.");
    double step = 1.0; // 1 meter per key press

    while (manual_mode && rclcpp::ok()) {
        char key = get_keyPress();
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);

            switch(key) {
                case 'U': y_ += step; break; // UP arrow 
                case 'D': y_ -= step; break; // DOWN
                case 'L': x_ -= step; break; // LEFT arrow → move left
                case 'R': x_ += step; break; // RIGHT arrow → move right
                case '\033': // ESC key
                    manual_mode = false;
                    RCLCPP_INFO(this->get_logger(), "Exiting manual mode...");
                    break;
            }

            if (key == 'U' || key == 'D' || key == 'L' || key == 'R') {
                RCLCPP_INFO(this->get_logger(), "Manual move: x=%.2f y=%.2f z=%.2f", x_, y_, z_);
            }
        }
        std::this_thread::sleep_for(50ms);
    }
}



    // ---------------- Members ----------------
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr leader_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr formation_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    double x_, y_, z_;
    int leader_id_;
    bool manual_mode = false;
    std::mutex pose_mutex_;
    std::map<std::string, rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr> setmode_clients_;
    std::map<std::string, rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr> arm_clients_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GroundControl>();

    std::thread input_thread([&node]() { node->run_input_loop(); });

    rclcpp::spin(node);
    input_thread.join();
    rclcpp::shutdown();
    return 0;
}
