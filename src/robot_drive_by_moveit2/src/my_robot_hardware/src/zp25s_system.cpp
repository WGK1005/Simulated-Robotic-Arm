#include "my_robot_hardware/zp25s_system.hpp"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <sstream>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace zp25s_hardware
{

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr int P_MIN = 500;
constexpr int P_MAX = 2500;
constexpr int READ_TIMEOUT_MS = 50;
}  // namespace

// ----------------------------------------------------------------------------
Zp25sSystem::~Zp25sSystem()
{
  close_serial();
}

hardware_interface::CallbackReturn Zp25sSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_ = info_.hardware_parameters.count("serial_port") > 0 ?
    info_.hardware_parameters.at("serial_port") : std::string("/dev/ttyACM0");
  if (info_.hardware_parameters.count("baud_rate") > 0) {
    baud_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  }
  if (info_.hardware_parameters.count("move_time_ms") > 0) {
    move_time_ms_ = std::stoi(info_.hardware_parameters.at("move_time_ms"));
  }

  joints_.clear();
  hw_commands_.clear();
  hw_positions_.clear();
  hw_names_.clear();

  for (const auto & j : info_.joints)
  {
    if (j.command_interfaces.empty() || j.state_interfaces.empty()) {
      continue;
    }
    // only joints with a position command interface are handled
    bool is_position = false;
    for (const auto & ci : j.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) {
        is_position = true;
      }
    }
    if (!is_position) {
      continue;
    }

    Zp25sJointConfig cfg;
    cfg.joint_name = j.name;

    std::string id_key = j.name + "_servo_id";
    if (info_.hardware_parameters.count(id_key) > 0) {
      cfg.servo_id = std::stoi(info_.hardware_parameters.at(id_key));
    }
    std::string dir_key = j.name + "_direction";
    if (info_.hardware_parameters.count(dir_key) > 0) {
      cfg.direction = std::stod(info_.hardware_parameters.at(dir_key));
    }

    joints_.push_back(cfg);
    hw_names_.push_back(j.name);
    hw_commands_.push_back(0.0);
    hw_positions_.push_back(0.0);
  }

  RCLCPP_INFO(rclcpp::get_logger("Zp25sSystem"),
    "configured %zu joints on %s", joints_.size(), port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> Zp25sSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joints_.size(); ++i) {
    state_interfaces.emplace_back(hw_names_[i], hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Zp25sSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joints_.size(); ++i) {
    command_interfaces.emplace_back(hw_names_[i], hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }
  return command_interfaces;
}

// ----------------------------------------------------------------------------
hardware_interface::CallbackReturn Zp25sSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!open_serial(port_, baud_)) {
    RCLCPP_ERROR(rclcpp::get_logger("Zp25sSystem"),
      "failed to open serial port %s", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // read current angle of every servo once so the arm starts from reality
  for (auto & j : joints_) {
    int p = 0;
    if (read_position(j.servo_id, &p)) {
      j.pos_rad = p_to_rad(j, p);
      j.cmd_rad = j.pos_rad;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("Zp25sSystem"), "activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zp25sSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ----------------------------------------------------------------------------
hardware_interface::return_type Zp25sSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }
  // round-robin: poll one servo per call (controller runs at ~50 Hz, so each
  // servo is refreshed every 5 calls ~ 100 ms). Fast enough for RViz/state.
  if (joints_.empty()) {
    return hardware_interface::return_type::OK;
  }
  poll_index_ %= joints_.size();
  auto & j = joints_[poll_index_];

  int p = 0;
  if (read_position(j.servo_id, &p)) {
    double rad = p_to_rad(j, p);
    // find matching hw_positions slot
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (hw_names_[i] == j.joint_name) {
        hw_positions_[i] = rad;
        j.pos_rad = rad;
        break;
      }
    }
  }
  ++poll_index_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Zp25sSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }
  for (size_t i = 0; i < joints_.size(); ++i) {
    auto & j = joints_[i];
    int p = rad_to_p(j, hw_commands_[i]);
    p = std::max(P_MIN, std::min(P_MAX, p));
    if (!send_position(j.servo_id, p, move_time_ms_)) {
      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("Zp25sSystem"),
        steady_clock, 2000, "servo %d write failed", j.servo_id);
    }
  }
  return hardware_interface::return_type::OK;
}

// ----------------------------------------------------------------------------
int Zp25sSystem::rad_to_p(const Zp25sJointConfig & j, double rad) const
{
  double deg = rad * 180.0 / PI;
  double p = j.zero_p + deg / j.deg_per_step * j.direction;
  return static_cast<int>(std::lround(p));
}

double Zp25sSystem::p_to_rad(const Zp25sJointConfig & j, int p) const
{
  double deg = (p - j.zero_p) * j.deg_per_step * j.direction;
  return deg * PI / 180.0;
}

// ----------------------------------------------------------------------------
bool Zp25sSystem::open_serial(const std::string & port, int baud)
{
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }
  struct termios tty;
  if (tcgetattr(fd_, &tty) != 0) {
    close_serial();
    return false;
  }
  speed_t speed = B115200;
  switch (baud) {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    default: speed = B115200; break;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;   // 8N1
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;  // no hardware flow control
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;      // 100 ms read timeout
  tcsetattr(fd_, TCSANOW, &tty);
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void Zp25sSystem::close_serial()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

// ----------------------------------------------------------------------------
bool Zp25sSystem::send_position(int servo_id, int p_value, int duration_ms)
{
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "#%03dP%04dT%04d!", servo_id, p_value, duration_ms);
  ssize_t n = ::write(fd_, cmd, std::strlen(cmd));
  if (n < 0) {
    return false;
  }
  // bus servos need a short gap between frames; give them one
  usleep(2000);
  return true;
}

bool Zp25sSystem::read_position(int servo_id, int * p_value_out)
{
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "#%03dPRAD!", servo_id);
  if (::write(fd_, cmd, std::strlen(cmd)) < 0) {
    return false;
  }

  // expect reply like "#001P1500!" (10 chars)
  char buf[32];
  size_t len = 0;
  struct pollfd pfd = {fd_, POLLIN, 0};
  int rc = ::poll(&pfd, 1, READ_TIMEOUT_MS);
  if (rc <= 0) {
    return false;
  }
  len = ::read(fd_, buf, sizeof(buf) - 1);
  if (len <= 0) {
    return false;
  }
  buf[len] = '\0';
  std::string s(buf);
  auto ppos = s.find('P');
  if (ppos == std::string::npos) {
    return false;
  }
  try {
    *p_value_out = std::stoi(s.substr(ppos + 1));
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

}  // namespace zp25s_hardware

PLUGINLIB_EXPORT_CLASS(
  zp25s_hardware::Zp25sSystem,
  hardware_interface::SystemInterface)
