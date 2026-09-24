#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/demo.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{@input-path    | assets/demo/demo  | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);
  std::atomic<bool> quit = false;

  auto t0 = std::chrono::steady_clock::now();

  // Planner 线程 — 模拟实车的高频决策下发
  auto plan_thread = std::thread([&]() {
    while (!quit) {
      if (!target_queue.empty()) {
        auto target = target_queue.front();
        auto plan = planner.plan(target, 22);  // 弹速 22m/s

        nlohmann::json data;
        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
        data["target_yaw"] = plan.target_yaw * 57.3;
        data["target_pitch"] = plan.target_pitch * 57.3;
        data["plan_yaw"] = plan.yaw * 57.3;
        data["plan_yaw_vel"] = plan.yaw_vel;
        data["plan_yaw_acc"] = plan.yaw_acc;
        data["plan_pitch"] = plan.pitch * 57.3;
        data["plan_pitch_vel"] = plan.pitch_vel;
        data["plan_pitch_acc"] = plan.pitch_acc;
        data["fire"] = plan.fire ? 1 : 0;
        data["control"] = plan.control ? 1 : 0;
        plotter.plot(data);

        tools::logger()->info(
          "[Planner] control={} fire={} yaw={:.2f}° pitch={:.2f}°",
          plan.control, plan.fire, plan.yaw * 57.3, plan.pitch * 57.3);
      }
      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  double last_t = -1;

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    auto yolo_start = std::chrono::steady_clock::now();
    solver.set_R_gimbal2world({w, x, y, z});
    auto armors = yolo.detect(img, frame_count);

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);

    auto aimer_start = std::chrono::steady_clock::now();

    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, push: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    // 可视化
    if (!targets.empty()) {
      auto target = targets.front();

      // 绿框：EKF 估计的装甲板位置
      for (const auto & xyza : target.armor_xyza_list()) {
        auto pts = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, pts, {0, 255, 0});
      }

      // 红框：MPC Planner 瞄准点
      auto aim_pts = solver.reproject_armor(
        planner.debug_xyza.head(3), planner.debug_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, aim_pts, {0, 0, 255});

      // 状态文本
      Eigen::VectorXd x = target.ekf_x();
      tools::draw_text(
        img,
        fmt::format("z={:.2f} vz={:.2f} w={:.1f} r={:.2f}", x[4], x[5], x[7], x[8]),
        {10, 30}, {255, 255, 255});
    }

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("mpc_test", img);
    if (cv::waitKey(30) == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();

  return 0;
}
