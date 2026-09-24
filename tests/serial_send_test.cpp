// 串口发送测试：持续发送指令，验证视觉→下位机的发送链路。
// 发送的 yaw/pitch 目标值按方波在 0 和幅值之间切换，下位机端能明显看到收到的数据在跳变。
//
// 编译（在项目根目录）：
//   g++ -std=c++17 -O2 \
//     -I. -I io/serial/include -I /usr/include/eigen3 -I /usr/include/opencv4 \
//     tests/serial_send_test.cpp \
//     io/gimbal/gimbal.cpp \
//     io/serial/src/serial.cc io/serial/src/impl/unix.cc \
//     io/serial/src/impl/list_ports/list_ports_linux.cc \
//     tools/crc.cpp tools/logger.cpp tools/math_tools.cpp \
//     -lspdlog -lfmt -lyaml-cpp -lpthread \
//     -o build/serial_send_test
//
// 运行：
//   mkdir -p logs
//   ./build/serial_send_test configs/standard3.yaml        # 不控制云台（安全，只验证发送）
//   ./build/serial_send_test configs/standard3.yaml 1      # 控制云台（云台会跟随，注意安全）

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

static volatile std::sig_atomic_t g_stop = 0;
static void handle_sigint(int) { g_stop = 1; }

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cerr << "用法: serial_send_test <config.yaml> [control=0|1]\n";
    return 1;
  }
  bool control = (argc >= 3) ? (std::atoi(argv[2]) != 0) : false;

  std::signal(SIGINT, handle_sigint);

  // 打开串口 + 启动后台读线程 + 等第一帧
  io::Gimbal gimbal(argv[1]);

  const float amp_yaw   = 0.20f;  // yaw 目标切换幅值 (rad)
  const float amp_pitch = 0.10f;  // pitch 目标切换幅值 (rad)
  const double period   = 2.0;    // 每 2 秒切换一次方波

  auto t0 = std::chrono::steady_clock::now();

  while (!g_stop) {
    auto t  = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration<double>(t - t0).count();

    // 方波：phase 在 0 / 1 之间切换，目标值随之在 0 和幅值之间跳变
    int phase = (static_cast<int>(dt / period)) % 2;
    float yaw   = phase ? amp_yaw : 0.0f;
    float pitch = phase ? amp_pitch : 0.0f;

    gimbal.send(control, false, yaw, 0, 0, pitch, 0, 0);

    tools::logger()->info(
      "SEND control={} mode={} yaw={:+.3f} pitch={:+.3f} (phase={})", control,
      control ? 1 : 0, yaw, pitch, phase);

    std::this_thread::sleep_for(10ms);
  }

  // 退出前发一帧"不控制"，避免下位机停在最后的目标位置
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  tools::logger()->info("Bye.");
  return 0;
}
