#pragma once

#include "bsp_can.h"
#include "comp_type.hpp"
#include "comp_ui.hpp"
#include "dev.hpp"
#include "dev_can.hpp"

#define DEV_CAP_FB_ID_BASE (0x211)
#define DEV_CAP_CTRL_ID_BASE (0x210)

namespace Device {
class Cap {
 public:
  /**
   * @brief 电容状态信息
   */
  typedef struct {
    float input_volt_; /* 电源输入电压 */
    float cap_volt_; /* 当前电压 ？*/
    float input_curr_;
    float target_power_;
    float percentage_; /* 电容当前能量的百分比  */
  } Info;

  /**
   * 电容输出信息
   */
  typedef struct {
    float power_limit_; /* 当前功率限制 */
  } Output;

  /**
   * 初始化参数
   */
  typedef struct {
    bsp_can_t can; /* 电容所在的 CAN 总线 */
    uint32_t index; /* 电容编号 */
  } Param;

  explicit Cap(Param& param);

  bool Update();

  bool Control() const;

  bool Offline();

  void Decode(Can::Pack& rx);

  float GetPercentage() const;

  Param param_;

  bool online_;

  uint32_t last_online_time_;

  uint32_t mailbox_;

  System::Queue<Can::Pack> control_feedback_ = System::Queue<Can::Pack>(1); // 返回数据队列

  System::Thread thread_; // 线程句柄

  Message::Topic<Cap::Info> info_tp_;

  Cap::Info info_;

  Cap::Output out_;
};
}  // namespace Device
