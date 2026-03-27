#include "hwt906p_driver/hwt906p_parser.hpp"

#include <cmath>

namespace hwt906p_driver
{

Hwt906pParser::FeedResult Hwt906pParser::feed(const std::vector<uint8_t> & chunk)
{
  buffer_.insert(buffer_.end(), chunk.begin(), chunk.end());
  FeedResult result;

  while (buffer_.size() >= kFrameLength) {
    if (buffer_.front() != kFrameHeader) {
      buffer_.erase(buffer_.begin());
      continue;
    }

    uint8_t checksum = 0;
    for (std::size_t i = 0; i < kFrameLength - 1; ++i) {
      checksum = static_cast<uint8_t>(checksum + buffer_[i]);
    }

    if (checksum != buffer_[kFrameLength - 1]) {
      buffer_.erase(buffer_.begin());
      continue;
    }

    const auto frame = buffer_;
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<long>(kFrameLength));
    result.stats.valid_frames++;
    result.stats.last_frame_type = frame[1];

    auto sample = parse_frame(frame.data());
    if (sample.has_value()) {
      result.samples.push_back(*sample);
      result.stats.emitted_samples++;
    } else {
      result.stats.unknown_frames++;
    }
  }

  return result;
}

void Hwt906pParser::reset()
{
  buffer_.clear();
  pending_state_ = PendingState{};
  fresh_acceleration_ = false;
  fresh_angular_velocity_ = false;
  fresh_euler_ = false;
  fresh_quaternion_ = false;
}

int16_t Hwt906pParser::to_int16(uint8_t lo, uint8_t hi)
{
  return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8U) | lo);
}

std::optional<ImuSample> Hwt906pParser::parse_frame(const uint8_t * frame)
{
  const auto frame_type = frame[1];
  const uint8_t * data = frame + 2;

  if (frame_type == 0x51) {
    const auto sample = maybe_emit_sample(frame_type);

    pending_state_.acceleration = Vector3{
      static_cast<double>(to_int16(data[0], data[1])) / 32768.0 * 16.0 * kGravity,
      static_cast<double>(to_int16(data[2], data[3])) / 32768.0 * 16.0 * kGravity,
      static_cast<double>(to_int16(data[4], data[5])) / 32768.0 * 16.0 * kGravity};
    pending_state_.temperature_c = static_cast<double>(to_int16(data[6], data[7])) / 100.0;
    fresh_acceleration_ = true;
    return sample;
  }

  if (frame_type == 0x52) {
    pending_state_.angular_velocity = Vector3{
      static_cast<double>(to_int16(data[0], data[1])) / 32768.0 * 2000.0 * kDegToRad,
      static_cast<double>(to_int16(data[2], data[3])) / 32768.0 * 2000.0 * kDegToRad,
      static_cast<double>(to_int16(data[4], data[5])) / 32768.0 * 2000.0 * kDegToRad};
    pending_state_.temperature_c = static_cast<double>(to_int16(data[6], data[7])) / 100.0;
    fresh_angular_velocity_ = true;
    return std::nullopt;
  }

  if (frame_type == 0x53) {
    pending_state_.euler_deg = Vector3{
      static_cast<double>(to_int16(data[0], data[1])) / 32768.0 * 180.0,
      static_cast<double>(to_int16(data[2], data[3])) / 32768.0 * 180.0,
      static_cast<double>(to_int16(data[4], data[5])) / 32768.0 * 180.0};
    fresh_euler_ = true;
    return std::nullopt;
  }

  if (frame_type == 0x54) {
    pending_state_.magnetic_field_raw = Vector3{
      static_cast<double>(to_int16(data[0], data[1])),
      static_cast<double>(to_int16(data[2], data[3])),
      static_cast<double>(to_int16(data[4], data[5]))};
    return std::nullopt;
  }

  if (frame_type == 0x59) {
    const double q0 = static_cast<double>(to_int16(data[0], data[1])) / kQuatScale;
    const double q1 = static_cast<double>(to_int16(data[2], data[3])) / kQuatScale;
    const double q2 = static_cast<double>(to_int16(data[4], data[5])) / kQuatScale;
    const double q3 = static_cast<double>(to_int16(data[6], data[7])) / kQuatScale;
    pending_state_.quaternion_xyzw = Quaternion{q1, q2, q3, q0};
    fresh_quaternion_ = true;
    return maybe_emit_sample(frame_type);
  }

  return std::nullopt;
}

std::optional<ImuSample> Hwt906pParser::maybe_emit_sample(uint8_t frame_type)
{
  if (!fresh_acceleration_ || !fresh_angular_velocity_) {
    return std::nullopt;
  }

  const bool should_emit_from_quaternion = frame_type == 0x59 && fresh_quaternion_;
  const bool should_emit_from_euler =
    frame_type == 0x51 && fresh_euler_ && !fresh_quaternion_;
  if (!should_emit_from_quaternion && !should_emit_from_euler) {
    return std::nullopt;
  }

  ImuSample sample;
  sample.acceleration = pending_state_.acceleration;
  sample.angular_velocity = pending_state_.angular_velocity;
  sample.temperature_c = pending_state_.temperature_c;

  if (should_emit_from_quaternion) {
    sample.quaternion_xyzw = pending_state_.quaternion_xyzw;
  } else {
    sample.euler_deg = pending_state_.euler_deg;
  }

  if (pending_state_.magnetic_field_raw.has_value()) {
    sample.magnetic_field_raw = pending_state_.magnetic_field_raw;
  }

  fresh_acceleration_ = false;
  fresh_angular_velocity_ = false;
  fresh_euler_ = false;
  fresh_quaternion_ = false;

  return sample;
}

}  // namespace hwt906p_driver
