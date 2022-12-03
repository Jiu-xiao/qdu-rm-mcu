#include "dev_cap.hpp"

#include "comp_utils.hpp"

#define CAP_RES (100) /* 电容数据分辨率 */

#define CAP_CUTOFF_VOLT \
  13.0f /* 电容截止电压，未接升压模块时要高于电调最低工作电压 */

using namespace Device;
/*
 * Feedback info: can_cap => rx_callback => control_feedback => Update => _info => info_topic(cap_info)
 * Output: can_out_topic => out_sub => out_ => Control
 */

/**
 * 电容初始化函数
 * @details 注册 Topic 和回调函数
 * @param param 初始化参数
 */
Cap::Cap(Cap::Param &param) : param_(param), info_tp_("cap_info") {
  /* 接受数据回调函数 */
  auto rx_callback = [](Can::Pack &rx, Cap *cap) {
    rx.index -= cap->param_.index;

    if (rx.index == 0) {
      cap->control_feedback_.OverwriteFromISR(rx);
    }

    return true;
  };

  Message::Topic<Can::Pack> cap_tp("can_cap");
  cap_tp.RegisterCallback(rx_callback, this);

  Can::Subscribe(cap_tp, this->param_.can, this->param_.index, 1);

  /* 电容线程函数 */
  auto cap_thread = [](Cap *cap) {
    Message::Subscriber out_sub("cap_out", cap->out_);

    while (1) {
      /* 读取裁判系统信息 */
      if (!cap->Update()) {
        /* 一定时间长度内接收不到电容反馈值，使电容离线 */
        cap->Offline();
      }
      cap->info_tp_.Publish(cap->info_);

      out_sub.DumpData();
      cap->Control();

      /* 运行结束，等待下一次唤醒 */
      System::Thread::Sleep(10);
    }
  };

  this->thread_.Create(cap_thread, this, "cap_thread", 256,
                       System::Thread::Medium);
}

/**
 * 从队列的数据包更新状态
 * @return 一定时间长度内接收不到电容反馈值，使电容离线
 */
bool Cap::Update() {
  Can::Pack rx;
  while (this->control_feedback_.Receive(rx, 0)) {
    this->Decode(rx);
    this->online_ = true;
    this->last_online_time_ = System::Thread::GetTick();
    return true;
  }
  return System::Thread::GetTick() - this->last_online_time_ <= 250;
}

/**
 * 解析返回的数据
 * @param rx 接收到原始数据包
 */
void Cap::Decode(Can::Pack &rx) {
  uint8_t *raw = rx.data;
  this->info_.input_volt_ = (float)((raw[1] << 8) | raw[0]) / (float)CAP_RES;
  this->info_.cap_volt_ = (float)((raw[3] << 8) | raw[2]) / (float)CAP_RES;
  this->info_.input_curr_ = (float)((raw[5] << 8) | raw[4]) / (float)CAP_RES;
  this->info_.target_power_ = (float)((raw[7] << 8) | raw[6]) / (float)CAP_RES;

  /* 更新电容状态和百分比 */
  this->info_.percentage_ = this->GetPercentage();
}

/**
 * 输出功率限制
 * @return 是否成功
 */
bool Cap::Control() const {
  auto pwr_lim = (uint16_t)(this->out_.power_limit_ * CAP_RES);

  Can::Pack tx_buff;

  tx_buff.index = DEV_CAP_CTRL_ID_BASE;

  tx_buff.data[0] = (pwr_lim >> 8) & 0xFF;
  tx_buff.data[1] = pwr_lim & 0xFF;

  return Can::SendPack(this->param_.can, tx_buff);
}

/**
 * @brief 电容离线时调用，作用为清空状态
 * @return always true
 */
bool Cap::Offline() {
  this->info_.cap_volt_ = 0;
  this->info_.input_curr_ = 0;
  this->info_.input_volt_ = 0;
  this->info_.target_power_ = 0;
  this->online_ = false;

  return true;
}

/**
 * @brief 获取电容当前能量的百分比
 * @details 电容能量 E = 0.5 * C * U^2，U为电容两侧电压，最低电压为截止电压
 * @return 返回表示电容当前能量的浮点数，范围在0到1之间
 */
float Cap::GetPercentage() const {
  const float c_max =
      this->info_.input_volt_ *
      this->info_.input_volt_; /* 电容电压最大值不超过输入电压 */
  const float c_cap = this->info_.cap_volt_ * this->info_.cap_volt_;
  const float c_min = CAP_CUTOFF_VOLT * CAP_CUTOFF_VOLT;
  float percentage = (c_cap - c_min) / (c_max - c_min);
  clampf(&percentage, 0.0f, 1.0f);
  return percentage;
}
