// 最小串口通信验证程序
// 只依赖 io::Gimbal（serial + spdlog + fmt + yaml-cpp + Eigen），不依赖相机/Ceres/OpenVINO。
// 用途：验证和下位机的串口收发是否打通。
//
// 编译（在项目根目录）：
//   g++ -std=c++17 -O2 \
//     -I. -I io/serial/include -I /usr/include/eigen3 -I /usr/include/opencv4 \
//     tests/serial_minimal.cpp \
//     io/gimbal/gimbal.cpp \
//     io/serial/src/serial.cc io/serial/src/impl/unix.cc \
//     io/serial/src/impl/list_ports/list_ports_linux.cc \
//     tools/crc.cpp tools/logger.cpp tools/math_tools.cpp \
//     -lspdlog -lfmt -lyaml-cpp -lpthread \
//     -o build/serial_minimal
//
// 运行：
//   mkdir -p logs && ./build/serial_minimal configs/standard3.yaml

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

// Ctrl+C 退出
static volatile std::sig_atomic_t g_stop = 0;
static void handle_sigint(int) { g_stop = 1; }

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cerr << "用法: serial_minimal <config.yaml>\n";
    return 1;
  }

  std::signal(SIGINT, handle_sigint);

  // 打开串口 + 启动后台读线程 + 阻塞等待下位机第一帧（CRC 校验通过才继续）
  io::Gimbal gimbal(argv[1]);

  while (!g_stop) {
    auto mode  = gimbal.mode();    // 下位机发来的模式
    auto state = gimbal.state();   // 下位机发来的状态
    auto t     = std::chrono::steady_clock::now();
    auto q     = gimbal.q(t);      // 插值取 t 时刻的四元数
    auto ypr   = tools::eulers(q, 2, 1, 0);

    tools::logger()->info(
      "mode={} bullet_speed={:.2f} yaw={:.2f} pitch={:.2f} yaw_vel={:.2f} pitch_vel={:.2f} "
      "count={}",
      gimbal.str(mode), state.bullet_speed, state.yaw, state.pitch, state.yaw_vel, state.pitch_vel,
      state.bullet_count);

    // 只发"不控制"心跳，验证视觉→下位机的发送链路，不接管云台。
    // 想测试云台响应时，把第一个参数改成 true（control），例如：
    //   gimbal.send(true, false, 0, 0, 0, 0, 0, 0);  // 接管云台，转到 yaw=0 pitch=0
    gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

    std::this_thread::sleep_for(10ms);
  }

  tools::logger()->info("Bye.");
  return 0;
}
