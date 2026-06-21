#include "robot_firmware/interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace robot_firmware {
    
    RobotInterface::RobotInterface() {

    }

    RobotInterface::~RobotInterface() {
        if (_arduino.IsOpen()) {
            try 
            {
                _arduino.Close();
            } 
            catch (...)  
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotInterface"), 
                    "Something went wrong while closing the connection with port " << _port);
            }
        }
    }

    hardware_interface::CallbackReturn RobotInterface::on_init(const hardware_interface::HardwareInfo& hardware_info) {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != CallbackReturn::SUCCESS)
            return result;
        
        try
        {
            _port = info_.hardware_parameters.at("port");
        }
        catch(const std::out_of_range& e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("RobotInterface"), "No serial port provided! Aborting.");
            return CallbackReturn::FAILURE;
        }
        
        _velocity_commands.reserve(info_.joints.size());
        _position_states.reserve(info_.joints.size());
        _velocity_states.reserve(info_.joints.size());
        _last_run = rclcpp::Clock().now();

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RobotInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(size_t i = 0; i < info_.joints.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
                hardware_interface::HW_IF_POSITION, &_position_states[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
                hardware_interface::HW_IF_VELOCITY, &_velocity_states[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for(size_t i = 0; i < info_.joints.size(); ++i) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, 
                hardware_interface::HW_IF_VELOCITY, &_velocity_commands[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn RobotInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("RobotInterface"), "Starting robot hardware...");
        _velocity_commands = {0.0, 0.0};
        _position_states   = {0.0, 0.0};
        _velocity_states   = {0.0, 0.0};

        try 
        {
            _arduino.Open(_port);
            _arduino.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotInterface"), 
                "Something went wrong while interacting with port " << _port);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("RobotInterface"), "Hardware started, Ready to take commands.");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RobotInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("RobotInterface"), "Stopping robot hardware...");

        if (_arduino.IsOpen()) {
            try
            {
                _arduino.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotInterface"), 
                    "Something went wrong while closing the port " << _port);
                return CallbackReturn::FAILURE;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("RobotInterface"), "Hardware stopped.");
        return CallbackReturn::SUCCESS;
    }

    // Ex: rp2.57, ln5.71
    // rp2.57 = right wheel, positive direction, vel = 2.57rad/s
    // ln5.71 = left wheel, negative direction, vel = 5.71rad/s

    hardware_interface::return_type RobotInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
        if (_arduino.IsDataAvailable()) {
            auto dt = (rclcpp::Clock().now() - _last_run).seconds();

            std::string message;
            _arduino.ReadLine(message);

            std::stringstream ss(message);
            std::string res;
            int multiplier = 1;

            while(std::getline(ss, res, ',')) {
                multiplier = res.at(1) == 'p' ? 1 : -1;
                
                if (res.at(0) == 'r') {
                    _velocity_states.at(0) = multiplier * std::stod(res.substr(2, res.size()));
                    _position_states.at(0) += _velocity_states.at(0) * dt;
                } else if (res.at(0) == 'l') {
                    _velocity_states.at(1) = multiplier * std::stod(res.substr(2, res.size()));
                    _position_states.at(1) += _velocity_states.at(1) * dt;
                }
            }
            _last_run = rclcpp::Clock().now();
        }

        return hardware_interface::return_type::OK;
    }
    
    hardware_interface::return_type RobotInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
        std::stringstream message_stream;
        char right_wheel_sign = _velocity_commands.at(0) >= 0 ? 'p' : 'n';
        char left_wheel_sign  = _velocity_commands.at(0) >= 0 ? 'p' : 'n';
        std::string compensate_zeros_right = "";
        std::string compensate_zeros_left  = "";

        if (std::abs(_velocity_commands.at(0)) <= 10.0)
            compensate_zeros_right = "0";
        else
            compensate_zeros_right = "";

        if (std::abs(_velocity_commands.at(1)) <= 10.0)
            compensate_zeros_left = "0";
        else
            compensate_zeros_left = "";
        
        message_stream << std::fixed << std::setprecision(2) 
            << 'r' << right_wheel_sign << compensate_zeros_right << std::abs(_velocity_commands.at(0))
            << ",l" << left_wheel_sign  << compensate_zeros_left  << std::abs(_velocity_commands.at(1)) << ',';
    
        try 
        {
            _arduino.Write(message_stream.str());
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("RobotInterface"), 
                "Something went wrong while sending the message " << message_stream.str()
                << " on the port " << _port);
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_firmware::RobotInterface, hardware_interface::SystemInterface)