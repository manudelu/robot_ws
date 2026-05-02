#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>

using std::placeholders::_1;

class SerialTransmitter : public rclcpp::Node 
{
    public:
        SerialTransmitter() 
            : Node("serial_transmitter")
        {
            declare_parameter<std::string>("port", "/dev/ttyUSB0");
            _port = get_parameter("port").as_string();
            _arduino.Open(_port);
            _arduino.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

            _sub = create_subscription<std_msgs::msg::String>(
                "serial_transmitter", 10, std::bind(&SerialTransmitter::msgCallback, this, _1));
        }
    
        ~SerialTransmitter()
        {
            _arduino.Close();
        }

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;
        LibSerial::SerialPort _arduino;
        std::string _port;

        void msgCallback(const std_msgs::msg::String &msg) {
            _arduino.Write(msg.data);
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialTransmitter>());
    rclcpp::shutdown();
    return 0;
}