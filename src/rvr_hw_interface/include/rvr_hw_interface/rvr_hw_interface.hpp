
#ifndef DRVR_RVR_HW_INTERFACE_HPP_
#define DRVR_RVR_HW_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "nav_msgs/msg/odometry.hpp"


namespace rvr_hw_interface {

class RVRInterface : public hardware_interface::SystemInterface {
public:
    RVRInterface();

    // Lifecycle

    // CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);

    // CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);

    // CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info);

    std::vector<hardware_interface::StateInterface> export_state_interfaces();

    std::vector<hardware_interface::CommandInterface> export_command_interfaces();

    hardware_interface::return_type read(const rclcpp::Time &t, const rclcpp::Duration &period);

    hardware_interface::return_type write(const rclcpp::Time &t, const rclcpp::Duration &period);

private:
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_imu_;
    int sockfd_;
    uint32_t prev_left, prev_right;
    double left_cal, right_cal;

    hardware_interface::CallbackReturn connect_to_rvr_server();
};

}

#endif
