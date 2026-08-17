#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

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
#include <cstdint>

#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define INT_ENABLE       0x38

#define ACCEL_XOUT_H     0x3B
#define ACCEL_YOUT_H     0x3D
#define ACCEL_ZOUT_H     0x3F

#define GYRO_XOUT_H      0x43
#define GYRO_YOUT_H      0x45
#define GYRO_ZOUT_H      0x47

#define PWR_MGMT_1       0x6B
#define WHO_AM_I         0x75

#define DEVICE_ADDRESS   0x68

class MPU6050Driver : public rclcpp::Node
{
public:
    MPU6050Driver()
        : Node("mpu6050_driver")
    {
        publisher_ = create_publisher<sensor_msgs::msg::Imu>(
            "/imu/mpu6050",
            10
        );

        if (!init_i2c()) {
            RCLCPP_FATAL(this->get_logger(), "MPU6050 initialization failed.");
            return;
        }

        calibrate_gyro();

        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MPU6050Driver::timer_callback, this)
        );
    }

    ~MPU6050Driver()
    {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    int fd_ = -1;
    bool connected_ = false;

    static constexpr double GRAVITY = 9.80665;

    // MPU6050 +/-2g
    static constexpr double ACCEL_SCALE = 16384.0;

    // MPU6050 +/-250 deg/s
    static constexpr double GYRO_SCALE = 131.0;

    // Accelerometer calibration
    double accel_bias_x_ = 0.0;
    double accel_bias_y_ = 0.0;
    double accel_bias_z_ = 3875.0;

    // Gyroscope calibration
    double gyro_bias_x_ = 0.0;
    double gyro_bias_y_ = 0.0;
    double gyro_bias_z_ = 0.0;

    bool init_i2c()
    {
        const char* filename = "/dev/i2c-1";

        connected_ = false;

        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }

        fd_ = open(filename, O_RDWR);

        if (fd_ < 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to open I2C bus: %s",
                filename
            );
            return false;
        }

        if (ioctl(fd_, I2C_SLAVE, DEVICE_ADDRESS) < 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to access MPU6050 at address 0x%02X",
                DEVICE_ADDRESS
            );

            close(fd_);
            fd_ = -1;

            return false;
        }

        int who_am_i = i2c_smbus_read_byte_data(fd_, WHO_AM_I);
        if (who_am_i < 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unable to read WHO_AM_I register."
            );
            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "MPU6050 WHO_AM_I = 0x%02X",
            who_am_i
        );

        if (who_am_i != 0x68) {
            RCLCPP_WARN(
                this->get_logger(),
                "Unexpected WHO_AM_I value: 0x%02X",
                who_am_i
            );
        }

        // Wake up MPU6050
        if (i2c_smbus_write_byte_data(fd_, PWR_MGMT_1, 0x01) < 0) {
            return configuration_error();
        }

        // DLPF enabled
        if (i2c_smbus_write_byte_data(fd_, CONFIG, 0x03) < 0) {
            return configuration_error();
        }

        // 100 Hz sample rate
        if (i2c_smbus_write_byte_data(fd_, SMPLRT_DIV, 9) < 0) {
            return configuration_error();
        }

        // Gyro +/-250 deg/s
        if (i2c_smbus_write_byte_data(fd_, GYRO_CONFIG, 0x00) < 0) {
            return configuration_error();
        }

        // Accelerometer +/-2g
        if (i2c_smbus_write_byte_data(fd_, ACCEL_CONFIG, 0x00) < 0) {
            return configuration_error();
        }

        // Enable data ready interrupt
        if (i2c_smbus_write_byte_data(fd_, INT_ENABLE, 0x01) < 0) {
            return configuration_error();
        }

        connected_ = true;

        RCLCPP_INFO(this->get_logger(), "MPU6050 successfully connected and configured.");

        return true;
    }

    bool configuration_error()
    {
        RCLCPP_ERROR(this->get_logger(), "MPU6050 register configuration failed.");

        connected_ = false;

        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }

        return false;
    }

    int16_t read_raw_data(uint8_t register_address)
    {
        if (!connected_ || fd_ < 0)
            return 0;

        int high = i2c_smbus_read_byte_data(fd_, register_address);
        int low = i2c_smbus_read_byte_data(fd_, register_address + 1);

        if (high < 0 || low < 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "I2C read failed at register 0x%02X",
                register_address
            );

            connected_ = false;
            return 0;
        }

        uint16_t value =
            (static_cast<uint16_t>(high) << 8) |
            static_cast<uint16_t>(low);

        return static_cast<int16_t>(value);
    }

    void calibrate_gyro()
    {
        constexpr int samples = 500;

        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;

        RCLCPP_WARN(this->get_logger(), "Gyro calibration started. KEEP ROBOT COMPLETELY STILL.");

        // Wait a little for sensor settling
        usleep(500000);

        for (int i = 0; i < samples; ++i) {
            sum_x += static_cast<double>(read_raw_data(GYRO_XOUT_H));
            sum_y += static_cast<double>(read_raw_data(GYRO_YOUT_H));
            sum_z += static_cast<double>(read_raw_data(GYRO_ZOUT_H));
            usleep(5000);
        }

        gyro_bias_x_ = sum_x / samples;
        gyro_bias_y_ = sum_y / samples;
        gyro_bias_z_ = sum_z / samples;

        RCLCPP_INFO(
            this->get_logger(),
            "GYRO BIAS RAW: x=%.2f y=%.2f z=%.2f",
            gyro_bias_x_,
            gyro_bias_y_,
            gyro_bias_z_
        );

        RCLCPP_INFO(this->get_logger(), "Gyro calibration completed.");
    }

    void timer_callback()
    {
        if (!connected_) {
            if (!init_i2c())
                return;
        }

        const int16_t ax_raw = read_raw_data(ACCEL_XOUT_H);
        const int16_t ay_raw = read_raw_data(ACCEL_YOUT_H);
        const int16_t az_raw = read_raw_data(ACCEL_ZOUT_H);
        const int16_t gx_raw = read_raw_data(GYRO_XOUT_H);
        const int16_t gy_raw = read_raw_data(GYRO_YOUT_H);
        const int16_t gz_raw = read_raw_data(GYRO_ZOUT_H);

        if (!connected_)
            return;

        const double ax_corrected = static_cast<double>(ax_raw) - accel_bias_x_;
        const double ay_corrected = static_cast<double>(ay_raw) - accel_bias_y_;
        const double az_corrected = static_cast<double>(az_raw) - accel_bias_z_;
        const double gx_corrected = static_cast<double>(gx_raw) - gyro_bias_x_;
        const double gy_corrected = static_cast<double>(gy_raw) - gyro_bias_y_;
        const double gz_corrected = static_cast<double>(gz_raw) - gyro_bias_z_;

        sensor_msgs::msg::Imu imu_msg;

        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link";

        // Accelerometer -> m/s^2
        imu_msg.linear_acceleration.x = (ax_corrected / ACCEL_SCALE) * GRAVITY;
        imu_msg.linear_acceleration.y = (ay_corrected / ACCEL_SCALE) * GRAVITY;
        imu_msg.linear_acceleration.z = (az_corrected / ACCEL_SCALE) * GRAVITY;

        // Gyroscope -> rad/s
        imu_msg.angular_velocity.x = (gx_corrected / GYRO_SCALE) * M_PI / 180.0;
        imu_msg.angular_velocity.y = (gy_corrected / GYRO_SCALE) * M_PI / 180.0;
        imu_msg.angular_velocity.z = (gz_corrected / GYRO_SCALE) * M_PI / 180.0;

        // Orientation not provided
        imu_msg.orientation_covariance[0] = -1.0;

        publisher_->publish(imu_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPU6050Driver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}