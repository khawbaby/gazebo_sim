#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

class RobotZeroHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
      return CallbackReturn::ERROR;

    RCLCPP_INFO(rclcpp::get_logger("RobotZeroHardware"), "Initializing hardware...");

    size_t n = info_.joints.size();

    hw_positions_.resize(n, 0.0);
    hw_velocities_.resize(n, 0.0);
    hw_commands_.resize(n, 0.0);

    serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);

    if (serial_fd_ < 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("RobotZeroHardware"),
                  "Serial not found. Running in dummy mode.");
      serial_fd_ = -1;
      return CallbackReturn::SUCCESS;
    }

    configure_serial(serial_fd_, B115200);

    RCLCPP_INFO(rclcpp::get_logger("RobotZeroHardware"), "Serial connected");

    return CallbackReturn::SUCCESS;
  }

  // --- EXPORT STATE INTERFACES ---
  // --- I can provide position + velocity ---  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
      state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    }

    return state_interfaces;
  }

  // --- EXPORT COMMAND INTERFACES ---
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
    }

    return command_interfaces;
  }

  // --- READ FROM ESP32 ---
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (serial_fd_ >= 0)
    {
      // write_serial("e\n");

      std::string response = read_line();

      std::string tag;
      int32_t left = 0, right = 0;
      std::stringstream ss(response);
      //RCLCPP_WARN(rclcpp::get_logger("RobotZeroHardware"), "RAW: '%s' (len=%zu)", response.c_str(), response.size());
      if (!(ss >> tag >> left >> right))
      {
        RCLCPP_WARN(rclcpp::get_logger("RobotZeroHardware"),
                    "Invalid serial data: '%s'", response.c_str());
        return return_type::OK;
      } else {
        RCLCPP_INFO(rclcpp::get_logger("RobotZeroHardware"),
                    "Parsed: tag='%s' left=%d right=%d", tag.c_str(), left, right);
      }

      hw_positions_[0] = left ;
      hw_positions_[1] = right;
    } else {
      double dt = 0.02;  // assume 50 Hz

      // simulate encoder feedback from commands
      hw_positions_[0] += hw_commands_[0] * dt;
      hw_positions_[1] += hw_commands_[1] * dt;

      hw_velocities_[0] = hw_commands_[0];
      hw_velocities_[1] = hw_commands_[1];
    }

    return return_type::OK;
  }

  // --- WRITE TO ESP32 ---
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    double left = hw_commands_[0] * 100.0;
    double right = hw_commands_[1] * 100.0;

    if (serial_fd_ >= 0)
    {

      std::string cmd = "o " + std::to_string(left) + " " + std::to_string(right) + "\n";
      RCLCPP_INFO(
        rclcpp::get_logger("RobotZeroHardware"),
        "CMD: %.3f %.3f | RAW: %s",
        left, right, cmd.c_str()
      );
      write_serial(cmd);

    } 
    

    return return_type::OK;
  }

private:
  int serial_fd_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // --- SERIAL HELPERS ---

  // void configure_serial(int fd, int baud)
  // {
  //   if (fd < 0) return;

  //   struct termios tty;
  //   memset(&tty, 0, sizeof tty);

  //   if (tcgetattr(fd, &tty) != 0)
  //   {
  //     perror("tcgetattr failed");
  //     return;
  //   }

  //   cfsetospeed(&tty, baud);
  //   cfsetispeed(&tty, baud);

  //   tty.c_cflag |= (CLOCAL | CREAD);
  //   tty.c_cflag &= ~CSIZE;
  //   tty.c_cflag |= CS8;
  //   tty.c_cflag &= ~PARENB;
  //   tty.c_cflag &= ~CSTOPB;
  //   tty.c_cflag &= ~CRTSCTS;

  //   tty.c_lflag = 0;
  //   tty.c_oflag = 0;
  //   tty.c_iflag = 0;

  //   tty.c_cc[VMIN]  = 0;   // non-blocking
  //   tty.c_cc[VTIME] = 1;   // 0.1 sec timeout

  //   if (tcsetattr(fd, TCSANOW, &tty) != 0)
  //   {
  //     perror("tcsetattr failed");
  //   }
  // }

  void configure_serial(int fd, int baud)
  {
    if (fd < 0) return;

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
      perror("tcgetattr failed");
      return;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    // Raw mode (IMPORTANT)
    cfmakeraw(&tty);

    // Enable receiver
    tty.c_cflag |= (CLOCAL | CREAD);

    // Non-blocking behavior
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      perror("tcsetattr failed");
    }
  }

  void write_serial(const std::string & msg)
  {
    if (serial_fd_ < 0) return;

    int n = ::write(serial_fd_, msg.c_str(), msg.size());
    if (n < 0)
    {
      perror("write failed");
    }
  }

  std::string read_line()
  {
    static std::string buffer;

    char c;
    while (true)
    {
      int n = ::read(serial_fd_, &c, 1);
    
      if (n < 0) {
          if (errno != EAGAIN && errno != EWOULDBLOCK) {
              perror("serial read error");
          }
      }
      
      if (n > 0)
      {
        if (c == '\n')
        {
          std::string line = buffer;
          buffer.clear();
          return line;
        }
        buffer += c;
      }
      else
      {
        break;  // no more data
      }
    }

    return "";  // no complete line yet
  }
};

// --- REGISTER PLUGIN ---
PLUGINLIB_EXPORT_CLASS(RobotZeroHardware, hardware_interface::SystemInterface)