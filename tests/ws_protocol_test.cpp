#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "crc.hpp"

namespace {

constexpr uint8_t SOF = 0x5AU;
constexpr size_t HEADER_SIZE = 4U;
constexpr size_t MAX_PAYLOAD_SIZE = 255U;

template <size_t PAYLOAD_SIZE>
std::array<uint8_t, HEADER_SIZE + PAYLOAD_SIZE + 2U> MakeFrame(
    uint8_t id, const std::array<uint8_t, PAYLOAD_SIZE>& payload) {
  std::array<uint8_t, HEADER_SIZE + PAYLOAD_SIZE + 2U> frame{};
  frame[0] = SOF;
  frame[1] = static_cast<uint8_t>(PAYLOAD_SIZE);
  frame[2] = id;
  frame[3] = LibXR::CRC8::Calculate(frame.data(), 3U);
  std::memcpy(frame.data() + HEADER_SIZE, payload.data(), PAYLOAD_SIZE);
  const uint16_t crc = LibXR::CRC16::Calculate(frame.data(), frame.size() - 2U);
  frame[frame.size() - 2U] = static_cast<uint8_t>(crc);
  frame[frame.size() - 1U] = static_cast<uint8_t>(crc >> 8U);
  return frame;
}

class FrameReader {
 public:
  bool Push(uint8_t byte) {
    if (size_ == 0U) {
      if (byte != SOF) return false;
      frame_[size_++] = byte;
      return false;
    }
    frame_[size_++] = byte;
    if (size_ == HEADER_SIZE) {
      if (!LibXR::CRC8::Verify(frame_.data(), HEADER_SIZE)) return Restart(byte);
      expected_size_ = HEADER_SIZE + frame_[1] + 2U;
    }
    if (size_ < HEADER_SIZE) return false;
    if (size_ < expected_size_) return false;
    const uint16_t received_crc =
        static_cast<uint16_t>(frame_[size_ - 2U]) |
        (static_cast<uint16_t>(frame_[size_ - 1U]) << 8U);
    if (size_ != expected_size_ ||
        LibXR::CRC16::Calculate(frame_.data(), size_ - 2U) != received_crc) {
      return Restart(byte);
    }
    accepted_id_ = frame_[2];
    payload_size_ = frame_[1];
    std::memcpy(payload_.data(), frame_.data() + HEADER_SIZE, payload_size_);
    size_ = 0U;
    return true;
  }
  uint8_t accepted_id() const { return accepted_id_; }
  size_t payload_size() const { return payload_size_; }
  const std::array<uint8_t, MAX_PAYLOAD_SIZE>& payload() const { return payload_; }

 private:
  bool Restart(uint8_t byte) {
    size_ = 0U;
    return Push(byte);
  }
  std::array<uint8_t, HEADER_SIZE + MAX_PAYLOAD_SIZE + 2U> frame_{};
  std::array<uint8_t, MAX_PAYLOAD_SIZE> payload_{};
  size_t size_ = 0U;
  size_t expected_size_ = 0U;
  size_t payload_size_ = 0U;
  uint8_t accepted_id_ = 0U;
};

template <size_t SIZE>
bool Feed(FrameReader& reader, const std::array<uint8_t, SIZE>& frame) {
  bool accepted = false;
  for (uint8_t byte : frame) accepted = reader.Push(byte) || accepted;
  return accepted;
}

}  // namespace

int main() {
  const std::array<uint8_t, 3U> payload{{0x11U, 0x22U, 0x33U}};
  const auto frame = MakeFrame(0x01U, payload);
  FrameReader reader;
  assert(Feed(reader, frame));
  assert(reader.accepted_id() == 0x01U);
  assert(reader.payload_size() == payload.size());
  assert(std::memcmp(reader.payload().data(), payload.data(), payload.size()) == 0);

  std::array<uint8_t, MAX_PAYLOAD_SIZE> maximum_payload{};
  maximum_payload.front() = 0xA5U;
  maximum_payload.back() = 0x5AU;
  const auto maximum_frame = MakeFrame(0x0EU, maximum_payload);
  assert(Feed(reader, maximum_frame));
  assert(reader.payload_size() == MAX_PAYLOAD_SIZE);
  assert(reader.payload().front() == 0xA5U);
  assert(reader.payload()[MAX_PAYLOAD_SIZE - 1U] == 0x5AU);

  auto bad_crc8 = frame;
  bad_crc8[3] ^= 0x01U;
  assert(!Feed(reader, bad_crc8));
  auto bad_crc16 = frame;
  bad_crc16.back() ^= 0x01U;
  assert(!Feed(reader, bad_crc16));
  assert(Feed(reader, frame));

  std::array<uint8_t, 3U> short_robot_payload{};
  const auto short_robot_frame = MakeFrame(0x01U, short_robot_payload);
  assert(Feed(reader, short_robot_frame));
  assert(reader.payload_size() < 60U);
}
