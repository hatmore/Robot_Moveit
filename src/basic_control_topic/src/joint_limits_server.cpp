#include <rclcpp/rclcpp.hpp>
#include <planning_sdk_msgs/srv/set_joint_limits.hpp>
#include <planning_sdk_msgs/srv/get_joint_limits.hpp>
#include <planning_sdk_msgs/msg/joint_limits_array.hpp>
#include <planning_sdk_msgs/msg/joint_limit.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>
#include <map>
#include <string>
#include <fstream>
#include <cmath>
#include <mutex>
#include <sstream>
#include <cstdlib>

struct JointLimitData {
    double min_position = -3.14159;
    double max_position = 3.14159;
    double max_velocity = 3.0;
    double effort_limit = 0.0;
};

class JointLimitsServer : public rclcpp::Node {
public:
    JointLimitsServer() : Node("joint_limits_server") {
        this->declare_parameter<bool>("load_persisted_on_startup", true);
        this->declare_parameter<std::string>("persist_file_path", getDefaultPersistPath());

        load_persisted_ = this->get_parameter("load_persisted_on_startup").as_bool();
        persist_path_ = this->get_parameter("persist_file_path").as_string();

        initJointNames();
        loadDefaultsFromURDF();
        loadDefaultsFromParams();

        if (load_persisted_) {
            loadPersistedOverrides();
        }

        auto qos = rclcpp::QoS(1).transient_local();
        limits_pub_ = this->create_publisher<planning_sdk_msgs::msg::JointLimitsArray>(
            "/algorithm/joint_limits/current", qos);

        set_srv_ = this->create_service<planning_sdk_msgs::srv::SetJointLimits>(
            "/algorithm/grasp_planning/set_joint_limits",
            std::bind(&JointLimitsServer::handleSet, this, std::placeholders::_1, std::placeholders::_2));

        get_srv_ = this->create_service<planning_sdk_msgs::srv::GetJointLimits>(
            "/algorithm/grasp_planning/get_joint_limits",
            std::bind(&JointLimitsServer::handleGet, this, std::placeholders::_1, std::placeholders::_2));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                current_joint_state_ = *msg;
            });

        publishCurrentLimits();
        RCLCPP_INFO(this->get_logger(), "Joint limits server started with %zu joints", limits_.size());
    }

private:
    std::string getDefaultPersistPath() {
        return "/ros2_ws/linden_robot_moveit/src/teleop_description_moveit_config/config/joint_limits_override.yaml";
    }

    void initJointNames() {
        joint_names_ = {
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint"
        };
        for (const auto& name : joint_names_) {
            limits_[name] = JointLimitData{};
        }
    }

    void loadDefaultsFromURDF() {
        this->declare_parameter<std::string>("robot_description", "");
        std::string urdf_string = this->get_parameter("robot_description").as_string();

        if (urdf_string.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "No robot_description parameter, position limits will use fallback defaults");
            return;
        }

        urdf::Model model;
        if (!model.initString(urdf_string)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse robot_description URDF");
            return;
        }

        for (const auto& name : joint_names_) {
            auto joint = model.getJoint(name);
            if (joint && joint->limits) {
                auto& lim = limits_[name];
                lim.min_position = joint->limits->lower;
                lim.max_position = joint->limits->upper;
                lim.max_velocity = joint->limits->velocity;
                lim.effort_limit = joint->limits->effort;
                urdf_hard_limits_[name] = {joint->limits->lower, joint->limits->upper};
                RCLCPP_INFO(this->get_logger(), "URDF [%s] pos=[%.4f, %.4f] vel=%.2f effort=%.2f",
                            name.c_str(), lim.min_position, lim.max_position,
                            lim.max_velocity, lim.effort_limit);
            } else {
                RCLCPP_WARN(this->get_logger(), "URDF joint [%s] not found or has no limits", name.c_str());
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded limits from URDF (robot_description)");
    }

    void loadDefaultsFromParams() {
        for (const auto& name : joint_names_) {
            auto& lim = limits_[name];
            std::string prefix = "robot_description_planning.joint_limits." + name;

            auto declare_if_needed = [this](const std::string& p, double def) -> double {
                if (!this->has_parameter(p)) {
                    this->declare_parameter<double>(p, def);
                }
                return this->get_parameter(p).as_double();
            };

            lim.max_velocity = declare_if_needed(prefix + ".max_velocity", lim.max_velocity);
            lim.min_position = declare_if_needed(prefix + ".min_position", lim.min_position);
            lim.max_position = declare_if_needed(prefix + ".max_position", lim.max_position);

            RCLCPP_INFO(this->get_logger(), "Final [%s] vel=%.2f pos=[%.4f, %.4f] effort=%.2f",
                        name.c_str(), lim.max_velocity, lim.min_position, lim.max_position, lim.effort_limit);
        }
    }

    void loadPersistedOverrides() {
        std::ifstream file(persist_path_);
        if (!file.is_open()) {
            RCLCPP_INFO(this->get_logger(), "No persisted overrides at %s", persist_path_.c_str());
            return;
        }

        std::string line;
        std::string current_joint;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;

            if (line.back() == ':' && line.find(' ') == std::string::npos) {
                current_joint = line.substr(0, line.size() - 1);
                continue;
            }

            if (current_joint.empty() || limits_.find(current_joint) == limits_.end()) continue;

            std::istringstream iss(line);
            std::string key;
            double val;
            iss >> key >> val;
            if (key.empty() || key.back() != ':') continue;
            key.pop_back();

            auto& lim = limits_[current_joint];
            if (key == "min_position")  lim.min_position = val;
            else if (key == "max_position")  lim.max_position = val;
            else if (key == "max_velocity")  lim.max_velocity = val;
            else if (key == "effort_limit")  lim.effort_limit = val;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded persisted overrides from %s", persist_path_.c_str());
    }

    void persistToFile() {
        auto dir = persist_path_.substr(0, persist_path_.rfind('/'));
        if (!dir.empty()) {
            std::string cmd = "mkdir -p " + dir;
            (void)system(cmd.c_str());
        }

        std::ofstream file(persist_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write persist file: %s", persist_path_.c_str());
            return;
        }

        file << "# Joint limits overrides (auto-generated)\n";
        for (const auto& [name, lim] : limits_) {
            file << name << ":\n";
            file << "  min_position: " << lim.min_position << "\n";
            file << "  max_position: " << lim.max_position << "\n";
            file << "  max_velocity: " << lim.max_velocity << "\n";
            file << "  effort_limit: " << lim.effort_limit << "\n";
        }

        RCLCPP_INFO(this->get_logger(), "Persisted limits to %s", persist_path_.c_str());
    }

    void publishCurrentLimits() {
        auto msg = planning_sdk_msgs::msg::JointLimitsArray();
        msg.timestamp = this->now();
        for (const auto& [name, lim] : limits_) {
            planning_sdk_msgs::msg::JointLimit jl;
            jl.joint_name    = name;
            jl.lower_limit   = lim.min_position;
            jl.upper_limit   = lim.max_position;
            jl.velocity_limit = lim.max_velocity;
            jl.effort_limit  = lim.effort_limit;
            msg.limits.push_back(jl);
        }
        limits_pub_->publish(msg);
    }

    // 将内部数据转为 JointLimit 消息
    planning_sdk_msgs::msg::JointLimit toMsg(const std::string& name, const JointLimitData& lim) {
        planning_sdk_msgs::msg::JointLimit msg;
        msg.joint_name = name;
        msg.lower_limit = lim.min_position;
        msg.upper_limit = lim.max_position;
        msg.velocity_limit = lim.max_velocity;
        msg.effort_limit = lim.effort_limit;
        return msg;
    }

    void handleSet(
        const planning_sdk_msgs::srv::SetJointLimits::Request::SharedPtr req,
        planning_sdk_msgs::srv::SetJointLimits::Response::SharedPtr res)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        res->timestamp = this->now();

        if (req->limits.empty()) {
            res->error_code = 3;
            res->error_message = "limits array is empty";
            return;
        }

        // 先全部校验，再统一写入
        for (const auto& jl : req->limits) {
            auto it = limits_.find(jl.joint_name);
            if (it == limits_.end()) {
                res->error_code = 1;
                res->error_message = "Unknown joint: " + jl.joint_name;
                RCLCPP_ERROR(this->get_logger(), "REJECT: %s", res->error_message.c_str());
                return;
            }

            // 检查不超出 URDF 硬限位
            auto hard_it = urdf_hard_limits_.find(jl.joint_name);
            if (hard_it != urdf_hard_limits_.end()) {
                double hard_min = hard_it->second.first;
                double hard_max = hard_it->second.second;
                if (jl.lower_limit < hard_min) {
                    res->error_code = 2;
                    res->error_message = jl.joint_name + " lower_limit " +
                        std::to_string(jl.lower_limit) + " < URDF hard min " + std::to_string(hard_min);
                    RCLCPP_ERROR(this->get_logger(), "REJECT: %s", res->error_message.c_str());
                    return;
                }
                if (jl.upper_limit > hard_max) {
                    res->error_code = 2;
                    res->error_message = jl.joint_name + " upper_limit " +
                        std::to_string(jl.upper_limit) + " > URDF hard max " + std::to_string(hard_max);
                    RCLCPP_ERROR(this->get_logger(), "REJECT: %s", res->error_message.c_str());
                    return;
                }
            }

            // 检查当前关节位置不超出新限位
            {
                std::lock_guard<std::mutex> state_lock(joint_state_mutex_);
                for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
                    if (current_joint_state_.name[i] == jl.joint_name) {
                        double pos = current_joint_state_.position[i];
                        if (pos < jl.lower_limit || pos > jl.upper_limit) {
                            res->error_code = 2;
                            res->error_message = jl.joint_name + " current pos " +
                                std::to_string(pos) + " exceeds new limits [" +
                                std::to_string(jl.lower_limit) + ", " + std::to_string(jl.upper_limit) + "]";
                            RCLCPP_ERROR(this->get_logger(), "REJECT: %s", res->error_message.c_str());
                            return;
                        }
                        break;
                    }
                }
            }
        }

        // 校验通过，全部写入
        for (const auto& jl : req->limits) {
            auto& lim = limits_[jl.joint_name];
            lim.min_position = jl.lower_limit;
            lim.max_position = jl.upper_limit;
            lim.max_velocity = jl.velocity_limit;
            lim.effort_limit = jl.effort_limit;
            RCLCPP_INFO(this->get_logger(), "Set [%s] pos=[%.3f,%.3f] vel=%.3f effort=%.3f",
                        jl.joint_name.c_str(), lim.min_position, lim.max_position,
                        lim.max_velocity, lim.effort_limit);
        }

        persistToFile();
        publishCurrentLimits();

        res->error_code = 0;
        res->error_message = "OK";
    }

    void handleGet(
        const planning_sdk_msgs::srv::GetJointLimits::Request::SharedPtr req,
        planning_sdk_msgs::srv::GetJointLimits::Response::SharedPtr res)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        res->timestamp = this->now();

        if (req->joint_names.empty()) {
            // 返回所有关节
            for (const auto& [name, lim] : limits_) {
                res->limits.push_back(toMsg(name, lim));
            }
            res->error_code = 0;
            res->error_message = "Returned all " + std::to_string(limits_.size()) + " joints";
        } else {
            for (const auto& name : req->joint_names) {
                auto it = limits_.find(name);
                if (it == limits_.end()) {
                    res->error_code = 1;
                    res->error_message = "Unknown joint: " + name;
                    res->limits.clear();
                    return;
                }
                res->limits.push_back(toMsg(it->first, it->second));
            }
            res->error_code = 0;
            res->error_message = "OK";
        }
    }

    std::vector<std::string> joint_names_;
    std::map<std::string, JointLimitData> limits_;
    std::map<std::string, std::pair<double, double>> urdf_hard_limits_;
    std::mutex mutex_;
    bool load_persisted_;
    std::string persist_path_;

    rclcpp::Publisher<planning_sdk_msgs::msg::JointLimitsArray>::SharedPtr limits_pub_;
    rclcpp::Service<planning_sdk_msgs::srv::SetJointLimits>::SharedPtr set_srv_;
    rclcpp::Service<planning_sdk_msgs::srv::GetJointLimits>::SharedPtr get_srv_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    sensor_msgs::msg::JointState current_joint_state_;
    std::mutex joint_state_mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointLimitsServer>());
    rclcpp::shutdown();
    return 0;
}
