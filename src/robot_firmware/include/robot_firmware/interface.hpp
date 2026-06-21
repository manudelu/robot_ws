#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <vector>
#include <string>
#include <libserial/SerialPort.h>

namespace robot_firmware {

    class RobotInterface : public hardware_interface::SystemInterface {
        public:
            RobotInterface();
            virtual ~RobotInterface();

            virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
            virtual hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
            virtual hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

            virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            virtual hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
            virtual hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        private:
            LibSerial::SerialPort _arduino;
            std::string _port;

            std::vector<double> _velocity_commands;
            std::vector<double> _position_states;
            std::vector<double> _velocity_states;

            rclcpp::Time _last_run;
    };

}
#endif