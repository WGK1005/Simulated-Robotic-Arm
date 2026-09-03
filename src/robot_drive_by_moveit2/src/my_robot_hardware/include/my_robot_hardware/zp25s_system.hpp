#ifndef ZP25S_SYSTEM_HPP_
#define ZP25S_SYSTEM_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <termios.h>
#include <string>
#include <vector>

namespace zp25s_hardware
{

// One actuated joint of the arm
struct Zp25sJointConfig
{
  std::string joint_name;
  int servo_id = 0;         // ZP25S bus ID (1..254)
  double direction = 1.0;   // +1/-1, URDF positive rotation vs increasing P
  double zero_p = 1500.0;   // P value read at URDF angle 0
  double deg_per_step = 0.1333;  // measured: 900 steps == 120 deg

  // live values
  double cmd_rad = 0.0;   // commanded (rad), written to servo
  double pos_rad = 0.0;   // read back (rad)
};

class Zp25sSystem : public hardware_interface::SystemInterface
{
public:
  Zp25sSystem() = default;
  virtual ~Zp25sSystem();

  // hardware_interface::SystemInterface API
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // serial port helpers (Linux termios)
  bool open_serial(const std::string & port, int baud);
  void close_serial();
  // write '#<id>PxxxxTxxxx!' ; read back angle via PRAD
  bool send_position(int servo_id, int p_value, int duration_ms);
  bool read_position(int servo_id, int * p_value_out);

  // conversion
  int rad_to_p(const Zp25sJointConfig & j, double rad) const;
  double p_to_rad(const Zp25sJointConfig & j, int p) const;

  // one servo is polled per read() call (round-robin); serial is slow
  size_t poll_index_ = 0;

  std::vector<Zp25sJointConfig> joints_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<std::string> hw_names_;

  std::string port_;
  int baud_ = 115200;
  int move_time_ms_ = 500;      // PxxxxTxxxx! duration for write
  int fd_ = -1;                 // serial fd
};

}  // namespace zp25s_hardware

#endif  // ZP25S_SYSTEM_HPP_
