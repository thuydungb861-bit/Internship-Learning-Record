#include "hwt906p_driver/hwt906p_parser.hpp"

#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace hwt906p_driver
{

using namespace std::chrono_literals;

namespace
{

constexpr std::array<double, 9> kDefaultOrientationCovariance = {
  5.250769e-09, 1.790648e-09, 0.0,
  1.790648e-09, 6.129790e-09, 0.0,
  0.0, 0.0, 1.0e-06};

constexpr std::array<double, 9> kDefaultAngularVelocityCovariance = {
  1.137772e-08, 1.599640e-08, 0.0,
  1.599640e-08, 2.523375e-08, 0.0,
  0.0, 0.0, 1.0e-08};

constexpr std::array<double, 9> kDefaultLinearAccelerationCovariance = {
  1.181765e-05, -1.705412e-07, 4.447220e-06,
  -1.705412e-07, 1.245834e-05, -2.611583e-06,
  4.447220e-06, -2.611583e-06, 3.182327e-05};

Quaternion euler_to_quaternion(const Vector3 & euler_deg)
{
  const double roll = euler_deg.x * kDegToRad;
  const double pitch = euler_deg.y * kDegToRad;
  const double yaw = euler_deg.z * kDegToRad;

  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  return Quaternion{
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy};
}

std::string preview_bytes(const std::vector<uint8_t> & chunk, std::size_t max_bytes)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  const auto count = std::min(chunk.size(), max_bytes);
  for (std::size_t i = 0; i < count; ++i) {
    if (i > 0) {
      stream << ' ';
    }
    stream << std::setw(2) << static_cast<int>(chunk[i]);
  }
  return stream.str();
}

}  // namespace

class Hwt906pNode : public rclcpp::Node
{
public:
  Hwt906pNode()
  : Node("hwt906p_node")
  {
    declare_parameter("port", "/dev/ttyUSB0");
    declare_parameter("baudrate", 115200);
    declare_parameter("frame_id", "imu_link");
    declare_parameter("poll_hz", 200.0);
    declare_parameter("magnetic_field_lsb_to_tesla", 1.0e-6);
    declare_parameter("reconnect_interval_sec", 1.0);
    declare_parameter("frame_timeout_sec", 2.0);
    declare_parameter("status_log_interval_sec", 2.0);
    declare_parameter("baudrate_candidates", std::vector<int64_t>{230400, 115200, 9600, 57600, 38400, 19200});

    port_ = get_parameter("port").as_string();
    baudrate_ = get_parameter("baudrate").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    poll_hz_ = get_parameter("poll_hz").as_double();
    reconnect_interval_sec_ = get_parameter("reconnect_interval_sec").as_double();
    frame_timeout_sec_ = get_parameter("frame_timeout_sec").as_double();
    status_log_interval_sec_ = get_parameter("status_log_interval_sec").as_double();
    mag_scale_ = get_parameter("magnetic_field_lsb_to_tesla").as_double();

    const auto baud_candidates_param = get_parameter("baudrate_candidates").as_integer_array();
    normalize_baud_candidates(baud_candidates_param);
    current_baudrate_ = baudrate_candidates_.front();

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 20);
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 20);

    const auto period = std::chrono::duration<double>(std::max(1.0 / poll_hz_, 0.001));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&Hwt906pNode::poll_serial, this));

    RCLCPP_INFO(
      get_logger(), "HWT906P C++ driver started for %s, baud candidates=%s, frame_id=%s",
      port_.c_str(), format_baud_candidates().c_str(), frame_id_.c_str());

    ensure_serial_connection(true);
  }

  ~Hwt906pNode() override
  {
    close_serial();
  }

private:
  void normalize_baud_candidates(const std::vector<int64_t> & values)
  {
    const std::vector<int64_t> combined = [&]() {
        std::vector<int64_t> result;
        result.push_back(baudrate_);
        result.insert(result.end(), values.begin(), values.end());
        return result;
      }();

    for (const auto baud : combined) {
      if (!is_supported_baudrate(static_cast<int>(baud))) {
        continue;
      }
      if (std::find(baudrate_candidates_.begin(), baudrate_candidates_.end(), baud) == baudrate_candidates_.end()) {
        baudrate_candidates_.push_back(static_cast<int>(baud));
      }
    }

    if (baudrate_candidates_.empty()) {
      throw std::runtime_error("No supported baudrate candidates were configured");
    }
  }

  static bool is_supported_baudrate(int baudrate)
  {
    switch (baudrate) {
      case 9600:
      case 19200:
      case 38400:
      case 57600:
      case 115200:
      case 230400:
      case 460800:
      case 921600:
        return true;
      default:
        return false;
    }
  }

  static speed_t to_speed(int baudrate)
  {
    switch (baudrate) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 921600: return B921600;
      default: throw std::runtime_error("Unsupported baudrate");
    }
  }

  std::string format_baud_candidates() const
  {
    std::ostringstream stream;
    stream << "[";
    for (std::size_t i = 0; i < baudrate_candidates_.size(); ++i) {
      if (i > 0) {
        stream << ", ";
      }
      stream << baudrate_candidates_[i];
    }
    stream << "]";
    return stream.str();
  }

  int open_serial(const std::string & port, int baudrate)
  {
    const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      throw std::runtime_error("open failed");
    }

    termios attrs {};
    if (tcgetattr(fd, &attrs) != 0) {
      ::close(fd);
      throw std::runtime_error("tcgetattr failed");
    }

    attrs.c_iflag = IGNPAR;
    attrs.c_oflag = 0;
    attrs.c_cflag = CLOCAL | CREAD | CS8;
    attrs.c_lflag = 0;
    cfsetispeed(&attrs, to_speed(baudrate));
    cfsetospeed(&attrs, to_speed(baudrate));
    attrs.c_cc[VMIN] = 0;
    attrs.c_cc[VTIME] = 0;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &attrs) != 0) {
      ::close(fd);
      throw std::runtime_error("tcsetattr failed");
    }
    return fd;
  }

  void close_serial()
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }

    if (connected_) {
      RCLCPP_WARN(get_logger(), "Lost connection to %s, waiting to reconnect", port_.c_str());
    }

    connected_ = false;
    parser_.reset();
    received_bytes_ = 0;
    valid_frames_ = 0;
    valid_samples_ = 0;
    unknown_frames_ = 0;
    last_frame_type_.reset();
    last_chunk_preview_.clear();
  }

  bool ensure_serial_connection(bool force = false)
  {
    if (serial_fd_ >= 0) {
      return true;
    }

    const auto now_tp = now();
    if (!force && (now_tp - last_reconnect_attempt_).seconds() < std::max(reconnect_interval_sec_, 0.1)) {
      return false;
    }
    last_reconnect_attempt_ = now_tp;

    try {
      serial_fd_ = open_serial(port_, current_baudrate_);
    } catch (const std::exception & exc) {
      const std::string error_text = exc.what();
      if (error_text != last_open_error_) {
        RCLCPP_WARN(
          get_logger(), "Unable to open %s @ %d: %s. Retrying every %.1fs", port_.c_str(),
          current_baudrate_, error_text.c_str(), reconnect_interval_sec_);
        last_open_error_ = error_text;
      }
      return false;
    }

    connected_ = true;
    last_open_error_.clear();
    last_valid_frame_ = now_tp;
    RCLCPP_INFO(
      get_logger(), "Connected to HWT906P on %s using baudrate %d", port_.c_str(),
      current_baudrate_);
    return true;
  }

  void advance_baudrate()
  {
    const auto previous = current_baudrate_;
    current_baud_index_ = (current_baud_index_ + 1) % baudrate_candidates_.size();
    current_baudrate_ = baudrate_candidates_[current_baud_index_];
    RCLCPP_WARN(
      get_logger(), "No valid HWT906P frames at %d baud, switching to %d", previous,
      current_baudrate_);
    close_serial();
  }

  void maybe_log_status(const rclcpp::Time & now_tp)
  {
    if ((now_tp - last_status_log_).seconds() < std::max(status_log_interval_sec_, 0.5)) {
      return;
    }
    last_status_log_ = now_tp;

    std::string frame_type_text = "none";
    if (last_frame_type_.has_value()) {
      std::ostringstream stream;
      stream << "0x" << std::hex << static_cast<int>(*last_frame_type_);
      frame_type_text = stream.str();
    }

    RCLCPP_INFO(
      get_logger(),
      "Serial status: baud=%d, bytes=%zu, valid_frames=%zu, emitted_samples=%zu, unknown_frames=%zu, last_frame_type=%s",
      current_baudrate_, received_bytes_, valid_frames_, valid_samples_, unknown_frames_,
      frame_type_text.c_str());
  }

  bool timed_out_since_last_valid_frame(const rclcpp::Time & now_tp) const
  {
    return (now_tp - last_valid_frame_).seconds() > std::max(frame_timeout_sec_, 0.5);
  }

  void poll_serial()
  {
    if (!ensure_serial_connection()) {
      return;
    }

    const auto now_tp = now();
    std::vector<uint8_t> chunk(4096);
    const auto read_count = ::read(serial_fd_, chunk.data(), chunk.size());

    if (read_count < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        if (timed_out_since_last_valid_frame(now_tp)) {
          advance_baudrate();
        }
        return;
      }

      RCLCPP_ERROR(get_logger(), "Serial read failed");
      close_serial();
      return;
    }

    if (read_count == 0) {
      if (timed_out_since_last_valid_frame(now_tp)) {
        advance_baudrate();
      }
      return;
    }

    chunk.resize(static_cast<std::size_t>(read_count));
    received_bytes_ += chunk.size();
    last_chunk_preview_ = preview_bytes(chunk, 16);

    auto result = parser_.feed(chunk);
    valid_frames_ += result.stats.valid_frames;
    unknown_frames_ += result.stats.unknown_frames;
    if (result.stats.last_frame_type.has_value()) {
      last_frame_type_ = result.stats.last_frame_type;
    }

    if (!result.samples.empty()) {
      valid_samples_ += result.samples.size();
      last_valid_frame_ = now_tp;
    } else if (timed_out_since_last_valid_frame(now_tp)) {
      RCLCPP_WARN(
        get_logger(), "No emitted samples at baud %d. Last chunk preview: %s", current_baudrate_,
        last_chunk_preview_.empty() ? "empty" : last_chunk_preview_.c_str());
      advance_baudrate();
      return;
    }

    maybe_log_status(now_tp);

    for (auto sample : result.samples) {
      publish_sample(sample);
    }
  }

  void publish_sample(const ImuSample & sample)
  {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now();
    imu_msg.header.frame_id = frame_id_;
    imu_msg.orientation_covariance = kDefaultOrientationCovariance;
    imu_msg.angular_velocity_covariance = kDefaultAngularVelocityCovariance;
    imu_msg.linear_acceleration_covariance = kDefaultLinearAccelerationCovariance;

    if (sample.quaternion_xyzw.has_value()) {
      imu_msg.orientation.x = sample.quaternion_xyzw->x;
      imu_msg.orientation.y = sample.quaternion_xyzw->y;
      imu_msg.orientation.z = sample.quaternion_xyzw->z;
      imu_msg.orientation.w = sample.quaternion_xyzw->w;
    } else if (sample.euler_deg.has_value()) {
      const auto q = euler_to_quaternion(*sample.euler_deg);
      imu_msg.orientation.x = q.x;
      imu_msg.orientation.y = q.y;
      imu_msg.orientation.z = q.z;
      imu_msg.orientation.w = q.w;
    } else {
      imu_msg.orientation_covariance[0] = -1.0;
    }

    if (sample.angular_velocity.has_value()) {
      imu_msg.angular_velocity.x = sample.angular_velocity->x;
      imu_msg.angular_velocity.y = sample.angular_velocity->y;
      imu_msg.angular_velocity.z = sample.angular_velocity->z;
    } else {
      imu_msg.angular_velocity_covariance[0] = -1.0;
    }

    if (sample.acceleration.has_value()) {
      imu_msg.linear_acceleration.x = sample.acceleration->x;
      imu_msg.linear_acceleration.y = sample.acceleration->y;
      imu_msg.linear_acceleration.z = sample.acceleration->z;
    } else {
      imu_msg.linear_acceleration_covariance[0] = -1.0;
    }

    imu_pub_->publish(imu_msg);

    if (sample.magnetic_field_raw.has_value()) {
      sensor_msgs::msg::MagneticField mag_msg;
      mag_msg.header = imu_msg.header;
      mag_msg.magnetic_field.x = sample.magnetic_field_raw->x * mag_scale_;
      mag_msg.magnetic_field.y = sample.magnetic_field_raw->y * mag_scale_;
      mag_msg.magnetic_field.z = sample.magnetic_field_raw->z * mag_scale_;
      mag_msg.magnetic_field_covariance = {1.0e-6, 0.0, 0.0, 0.0, 1.0e-6, 0.0, 0.0, 0.0, 1.0e-6};
      mag_pub_->publish(mag_msg);
    }
  }

  std::string port_;
  int baudrate_ {115200};
  std::string frame_id_ {"imu_link"};
  double poll_hz_ {200.0};
  double reconnect_interval_sec_ {1.0};
  double frame_timeout_sec_ {2.0};
  double status_log_interval_sec_ {2.0};
  double mag_scale_ {1.0e-6};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Hwt906pParser parser_;
  int serial_fd_ {-1};
  bool connected_ {false};
  std::string last_open_error_;
  rclcpp::Time last_reconnect_attempt_ {0, 0, RCL_ROS_TIME};
  std::size_t current_baud_index_ {0};
  std::vector<int> baudrate_candidates_;
  int current_baudrate_ {115200};
  std::size_t received_bytes_ {0};
  std::size_t valid_frames_ {0};
  std::size_t valid_samples_ {0};
  std::size_t unknown_frames_ {0};
  rclcpp::Time last_valid_frame_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_log_ {0, 0, RCL_ROS_TIME};
  std::optional<uint8_t> last_frame_type_;
  std::string last_chunk_preview_;
};

}  // namespace hwt906p_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<hwt906p_driver::Hwt906pNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(rclcpp::get_logger("hwt906p_node"), "HWT906P node stopped: %s", exc.what());
  }
  rclcpp::shutdown();
  return 0;
}
