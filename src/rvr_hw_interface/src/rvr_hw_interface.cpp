#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#include <unistd.h>

#include "rvr_hw_interface/rvr_hw_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rvr_hw_interface {

    RVRInterface::RVRInterface() {
        sockfd_ = 0;
    }

    hardware_interface::CallbackReturn RVRInterface::on_init(const hardware_interface::HardwareInfo &info) {
        const hardware_interface::CallbackReturn parent_init = SystemInterface::on_init(info);

        if (parent_init != hardware_interface::CallbackReturn::SUCCESS) {
            return parent_init;
        }

        for (const hardware_interface::ComponentInfo &joint: info_.joints) {
            if (joint.command_interfaces.size() != 2) {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RVRInterface"),
                    "Joint '%s' has %zu command interfaces. 2 expected.", 
                        joint.name.c_str(),
                        joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RVRInterface"),
                    "Joint '%s' is %s interface. %s expected.",
                        joint.name.c_str(),
                        joint.command_interfaces[0].name.c_str(),
                        hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Ignoring Position

            if (joint.state_interfaces.size() != 2) {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RVRInterface"),
                    "Joint '%s' has %zu state interfaces. 2 expected.",
                        joint.name.c_str(),
                        joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RVRInterface"),
                    "Joint '%s' has '%s' as second state interface. %s expected.",
                        joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(),
                        hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Ignoring Position
        }
        left_cal = 3976.7;
        right_cal = 3843.7;

        RCLCPP_INFO(rclcpp::get_logger("RVRInterface"), "Joints %ld", info_.joints.size());
        hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RVRInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RVRInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto i = 0u; i< info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn RVRInterface::connect_to_rvr_server() {
        struct sockaddr_un serv_addr;
        bzero((char *)&serv_addr,sizeof(serv_addr));
        serv_addr.sun_family = AF_UNIX;
        strcpy(serv_addr.sun_path, "/tmp/rvr.socket");
        int servlen = strlen(serv_addr.sun_path) + sizeof(serv_addr.sun_family);
        if ((sockfd_ = socket(AF_UNIX, SOCK_STREAM,0)) < 0) {
            RCLCPP_WARN(
                rclcpp::get_logger("RVRInterface"),
                "Failed to create RVR control socket");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (connect(sockfd_, (struct sockaddr *)&serv_addr, servlen) < 0) {
            RCLCPP_WARN(
                rclcpp::get_logger("RVRInterface"),
                "Failed to connect to RVR control server");
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RVRInterface::on_activate(const rclcpp_lifecycle::State &) {
        for (auto i = 0u; i < hw_velocities_.size(); i++) {
            if (std::isnan(hw_velocities_[i])) {
                hw_velocities_[i] = 0;
            }
            if (std::isnan(hw_commands_[i])) {
                hw_commands_[i] = 0;
            }
        }
        int retry=10;
        while (connect_to_rvr_server() != hardware_interface::CallbackReturn::SUCCESS) {
            retry--;
            if (retry == 0) {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RVRInterface"),
                    "Failed to connect to RVR control server. Out of retries");
                return hardware_interface::CallbackReturn::ERROR;
            }
            sleep(1);
        }
        RCLCPP_INFO(rclcpp::get_logger("RVRInterface"), "Socket: %d", sockfd_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RVRInterface::on_deactivate(const rclcpp_lifecycle::State &) {
        // Send stop commands
        ::write(sockfd_, "QUIT", 4);
        close(sockfd_);
        sockfd_ = 0;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RVRInterface::read(const rclcpp::Time &, const rclcpp::Duration &period) {
        ::write(sockfd_, "RVEL", 4);
        uint32_t left, right;
        ::read(sockfd_, &left, sizeof(uint32_t));
        ::read(sockfd_, &right, sizeof(uint32_t));
        // RCLCPP_INFO(rclcpp::get_logger("RVRInterface"), "Got Left %u Right %u Period %.5f", left, right, period.seconds());
        if (prev_left == 0 && prev_right == 0) {
            hw_velocities_[0] = 0.0;
            hw_velocities_[1] = 0.0;

        } else {
            hw_velocities_[0] = (left-prev_left)/left_cal/period.seconds();
            hw_velocities_[1] = (right-prev_right)/right_cal/period.seconds();
        }
        hw_positions_[0] = 0;
        hw_positions_[1] = 0;
        prev_left = left;
        prev_right = right;
        // RCLCPP_INFO(rclcpp::get_logger("RVRInterface"), "Velocity Left %.5f Right %.5f", hw_velocities_[0], hw_velocities_[1]);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RVRInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
        // Send the hw_commands_ as a msg to the RVR
        // Use the unix socket to send the current velocities
        ::write(sockfd_, "WVEL", 4);
        ::write(sockfd_, &hw_commands_[0], sizeof(double));
        ::write(sockfd_, &hw_commands_[1], sizeof(double));
        // RCLCPP_INFO(rclcpp::get_logger("RVRInterface"), "Wrote x %.5f y %.5f", hw_commands_[0], hw_commands_[1]);
        return hardware_interface::return_type::OK;
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rvr_hw_interface::RVRInterface, hardware_interface::SystemInterface)
