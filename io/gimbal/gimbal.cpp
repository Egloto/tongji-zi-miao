#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(921600);
    serial_.setTimeout(serial::Timeout::max(), 20, 0, 20, 0);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);//专门读取进程，线程a在这里启动，

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}//返回i状态

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
    tools::logger()->warn("[Gimbal] INF/NAN! yaw={} pitch={}", yaw, pitch);
    return;
  }
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  if (control) {
    auto raw = reinterpret_cast<const uint8_t *>(&tx_data_);
    tools::logger()->info(
      "[Send] {:02x}{:02x} {:02x} "
      "y:{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x} "
      "v:{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x} "
      "a:{:02x}{:02x}{:02x}{:02x} "
      "p:{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x} "
      "v:{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x} "
      "a:{:02x}{:02x}{:02x}{:02x} "
      "crc:{:02x}{:02x} | yaw={:.3f}° pitch={:.3f}°",
      raw[0], raw[1], raw[2],
      raw[3], raw[4], raw[5], raw[6], raw[7], raw[8], raw[9], raw[10],
      raw[11], raw[12], raw[13], raw[14], raw[15], raw[16], raw[17], raw[18],
      raw[19], raw[20], raw[21], raw[22],
      raw[23], raw[24], raw[25], raw[26], raw[27], raw[28], raw[29], raw[30],
      raw[31], raw[32], raw[33], raw[34], raw[35], raw[36], raw[37], raw[38],
      raw[39], raw[40], raw[41], raw[42],
      raw[43], raw[44],
      yaw * 57.3, pitch * 57.3);
  }

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()//读取进程，读取串口数据的进程
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;//数据头不对，继续读取

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      auto computed =
        tools::get_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_) - 2);
      auto received = rx_data_.crc16;
      auto mode = rx_data_.mode;
      auto yaw = rx_data_.yaw;
      auto pitch = rx_data_.pitch;
      auto bcnt = rx_data_.bullet_count;
      auto * raw = reinterpret_cast<const uint8_t *>(&rx_data_);
      tools::logger()->info(
        "[Gimbal] CRC16 fail: computed=0x{:04x} received=0x{:04x} | mode={} yaw={:.2f} pitch={:.2f} "
        "bcnt={} | raw: {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
        computed, received, mode, yaw, pitch, bcnt, raw[0], raw[1], raw[2], raw[3], raw[4], raw[5],
        raw[6], raw[7]);
      continue;
    }

    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;//state_结构体变量赋值

    {
      auto mode = rx_data_.mode;
      auto yaw = rx_data_.yaw;
      auto pitch = rx_data_.pitch;
      auto vyaw = rx_data_.yaw_vel;
      auto vpitch = rx_data_.pitch_vel;
      auto bspeed = rx_data_.bullet_speed;
      auto bcnt = rx_data_.bullet_count;
 //     tools::logger()->info(
 //       "[Gimbal] OK: mode={} yaw={:.2f} pitch={:.2f} vyaw={:.2f} vpitch={:.2f} bspeed={:.1f} bcnt={}",
  //      mode, yaw, pitch, vyaw, vpitch, bspeed, bcnt);
    }

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      serial_.flushInput();
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io