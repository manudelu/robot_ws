#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>
#include <chrono>

using namespace std::chrono_literals;

class SerialReceiver : public rclcpp::Node
{
    public:
        SerialReceiver()
            : Node("serial_receiver")
        {
            declare_parameter<std::string>("port", "/dev/ttyUSB0");
            _port = get_parameter("port").as_string();
            _arduino.Open(_port);
            _arduino.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

            _pub = create_publisher<std_msgs::msg::String>("serial_receiver", 10);
            _timer = create_wall_timer(10ms, std::bind(&SerialReceiver::timerCallback, this));
        }

        ~SerialReceiver()
        {
            _arduino.Close();
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;
        rclcpp::TimerBase::SharedPtr _timer;
        LibSerial::SerialPort _arduino;
        std::string _port;
        
        void timerCallback() {
            if(rclcpp::ok() && _arduino.IsDataAvailable()) {
                auto message = std_msgs::msg::String();
                _arduino.ReadLine(message.data);
                _pub->publish(message);
            }
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialReceiver>());
    rclcpp::shutdown();
    return 0;
}
