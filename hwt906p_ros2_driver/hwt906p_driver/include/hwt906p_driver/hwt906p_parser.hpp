#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace hwt906p_driver
{

constexpr double kGravity = 9.80665;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kQuatScale = 32768.0;

struct Vector3
{
  double x {0.0};
  double y {0.0};
  double z {0.0};
};

struct Quaternion
{
  double x {0.0};
  double y {0.0};
  double z {0.0};
  double w {1.0};
};

struct ImuSample
{
  std::optional<Vector3> acceleration;
  std::optional<Vector3> angular_velocity;
  std::optional<Vector3> euler_deg;
  std::optional<Quaternion> quaternion_xyzw;
  std::optional<Vector3> magnetic_field_raw;
  std::optional<double> temperature_c;
};

struct ParseStats
{
  std::size_t valid_frames {0};
  std::size_t emitted_samples {0};
  std::size_t unknown_frames {0};
  std::optional<uint8_t> last_frame_type;
};

class Hwt906pParser
{
public:
  static constexpr uint8_t kFrameHeader = 0x55;
  static constexpr std::size_t kFrameLength = 11;

  struct FeedResult
  {
    std::vector<ImuSample> samples;
    ParseStats stats;
  };

  FeedResult feed(const std::vector<uint8_t> & chunk);
  void reset();

private:
  struct PendingState
  {
    std::optional<Vector3> acceleration;
    std::optional<Vector3> angular_velocity;
    std::optional<Vector3> euler_deg;
    std::optional<Quaternion> quaternion_xyzw;
    std::optional<Vector3> magnetic_field_raw;
    std::optional<double> temperature_c;
  };

  static int16_t to_int16(uint8_t lo, uint8_t hi);
  std::optional<ImuSample> parse_frame(const uint8_t * frame);
  std::optional<ImuSample> maybe_emit_sample(uint8_t frame_type);

  std::vector<uint8_t> buffer_;
  PendingState pending_state_;
  bool fresh_acceleration_ {false};
  bool fresh_angular_velocity_ {false};
  bool fresh_euler_ {false};
  bool fresh_quaternion_ {false};
};

}  // namespace hwt906p_driver
