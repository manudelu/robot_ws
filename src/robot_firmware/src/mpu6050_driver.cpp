#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// sudo apt-get install libi2c-dev i2c-tools
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <memory>
#include <chrono>
#include <cmath>

#define PWR_MGMT_1     0x6B
#define SMPLRT_DIV     0x19
#define CONFIG         0x1A
#define GYRO_CONFIG    0x1B
#define INT_ENABLE     0x38
#define ACCEL_XOUT_H   0x3B
#define ACCEL_YOUT_H   0x3D
#define ACCEL_ZOUT_H   0x3F
#define GYRO_XOUT_H    0x43
#define GYRO_YOUT_H    0x45
#define GYRO_ZOUT_H    0x47
#define DEVICE_ADDRESS 0x68

class MPU6050_Driver : public rclcpp::Node {
    public:
        MPU6050_Driver()
            : Node("mpu6050_driver") 
        {
            _pub = create_publisher<sensor_msgs::msg::Imu>("/imu/mpu6050", 10);
            
            if (init_I2C())
                RCLCPP_INFO(this->get_logger(), "MPU6050 driver initialized successfully.");
            else
                RCLCPP_FATAL(this->get_logger(), "MPU6050 initialization failed! Shutting down node.");
            
             _timer = create_wall_timer(std::chrono::milliseconds(10), std::bind(&MPU6050_Driver::timerCallback, this));
        }

        ~MPU6050_Driver()
        {
            if (_fd < 0)
                close(_fd);
        }

    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _pub;
        rclcpp::TimerBase::SharedPtr _timer;
        bool _is_connected = false;
        int _fd = -1;

        void timerCallback() {
            if (!_is_connected)
                if (!init_I2C())
                    return;

            auto _imuMsg = sensor_msgs::msg::Imu();
            _imuMsg.header.frame_id = "base_footprint";
            _imuMsg.header.stamp = this->get_clock()->now();

            // Accelerations [m/s^2]
            _imuMsg.linear_acceleration.x = (double)read_raw_data(ACCEL_XOUT_H) / 1670.13;
            _imuMsg.linear_acceleration.y = (double)read_raw_data(ACCEL_YOUT_H) / 1670.13;
            _imuMsg.linear_acceleration.z = (double)read_raw_data(ACCEL_ZOUT_H) / 1670.13;

            // Angular Velocities [rad/s]
            _imuMsg.angular_velocity.x = (double)read_raw_data(GYRO_XOUT_H) / 7509.55;
            _imuMsg.angular_velocity.y = (double)read_raw_data(GYRO_YOUT_H) / 7509.55;
            _imuMsg.angular_velocity.z = (double)read_raw_data(GYRO_ZOUT_H) / 7509.55;

            _pub->publish(_imuMsg);
        }

        bool init_I2C() {
            // The Raspberry Pi 5 uses I2C adapter bus1 by default
            const char* filename = "/dev/i2c-1";

            if (_fd >= 0) {
                close(_fd);
                _fd = -1;
            }

            _fd = open(filename, O_RDWR);
            if (_fd < 0) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to open I2C bus: %s", filename);
                return false;
            }

            if(ioctl(_fd, I2C_SLAVE, DEVICE_ADDRESS) < 0) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to acquire bus access to slave address 0x%02X.", DEVICE_ADDRESS);
                close(_fd);
                _fd = -1;
                return false;
            }

            // CONFIG: DLPF_CFG = 0 -> Gyroscope Output Rate = 8kHz
            // SMPLRT_DIV: Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) = 8000[Hz] / (1 + 7) = 1kHz
            // GYRO_CONFIG: FS_SEL = 3 -> Gyroscope FUll Scale Range = +/- 2000°/s
            // PWR_MGMT_1: Wakes up the MPU6050 and sets the gyroscope x-axis as reference clock (CLKSEL = 1)
            // INT_ENABLE: Enable Interrupt PIN  (DATA_RDY_EN = 1)
            if (i2c_smbus_write_byte_data(_fd, CONFIG, 0) < 0 ||
                i2c_smbus_write_byte_data(_fd, SMPLRT_DIV, 7) < 0 ||
                i2c_smbus_write_byte_data(_fd, GYRO_CONFIG, 24) < 0 ||
                i2c_smbus_write_byte_data(_fd, PWR_MGMT_1, 1) < 0 ||
                i2c_smbus_write_byte_data(_fd, INT_ENABLE, 1) < 0) 
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "MPU6050 register configuration failed.");
                close(_fd);
                _fd = -1;
                return false;
            }

            _is_connected = true;
            RCLCPP_INFO(this->get_logger(), "MPU6050 successfully connected and configured.");
            return true;
        }

        int16_t read_raw_data(uint8_t addr) {
            if (!_is_connected || _fd < 0)
                return 0;

            // Reads both High and Low byte
            int32_t raw = i2c_smbus_read_word_data(_fd, addr);

            if (raw < 0) {
                RCLCPP_WARN(this->get_logger(), "I2C read failed at register 0x%02X", addr);
                _is_connected = false;
                return 0;
            }

            // Big-Endian -> Little-Endian (invert byte order)
            return __builtin_bswap16((int16_t)raw);
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6050_Driver>());
    rclcpp::shutdown();
}