# exit cpp

## 说明

这是一个简单的退出 C++ 程序的工具类，按 `Ctrl+C` 即可退出程序。

>**原理：** 你按 Ctrl+C → 终端驱动 → 内核向当前进程发送 SIGINT（信号编号 2）

## 代码

```cpp
#include "exiter.hpp"

#include <csignal>
#include <stdexcept>

namespace tools
{
bool exit_ = false;
bool exiter_inited_ = false;

Exiter::Exiter()
{
  if (exiter_inited_) throw std::runtime_error("Multiple Exiter instances!");
  std::signal(SIGINT, [](int) { exit_ = true; });
  exiter_inited_ = true;//计数，如果创建了exiter，if那边会停
}

bool Exiter::exit() const { return exit_; }

}  // namespace tools
```
> std::signal(SIGINT, [](int) { exit_ = true; });
简单来说，你按 Ctrl+C 会触发 SIGINT 信号。 然后执行 exit=true
这里有lambda的使用
就是函数的简化使用 
#
std::signal(SIGINT, [](int) { exit_ = true; });
#
void handle_signal(int sig) {
    exit_ = true;
}

// 再把函数名传进去
std::signal(SIGINT, handle_signal);




 // 可视化
      if (!targets.empty()) {
        auto target = targets.front();
        for (const auto & xyza : target.armor_xyza_list()) {
          auto pts = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(img, pts, {0, 255, 0});
        }
        auto aim_pts = solver.reproject_armor(
          planner.debug_xyza.head(3), planner.debug_xyza[3], target.armor_type, target.name);
        tools::draw_points(img, aim_pts, {0, 0, 255});
      }


      cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("standard_mpc", img);
    if (cv::waitKey(1) == 'q') break;测试提交
