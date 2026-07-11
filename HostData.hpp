#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - host_euler_topic_name: "target_eulr"
  - host_chassis_data_topic_name: "host_chassis_data"
  - host_fire_topic_name: "host_fire_notify"
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include "CMD.hpp"
#include "app_framework.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

/**
 * @brief 上位机数据接入模块
 * @details 将上位机发送的云台、底盘、发射命令转换为 CMD::Data 并喂给 CMD 模块。
 */
class HostData : public LibXR::Application {
 public:
  struct HostChassisTarget {
    float vx;
    float vy;
    float w;
  };

  struct LauncherCMD {
    bool isfire;
  };

  struct HostGimbalTarget {
    float rol, pit, yaw;
    float rol_dot, pit_dot, yaw_dot;
    float rol_ddot, pit_ddot, yaw_ddot;
  };

  /**
   * @brief 构造 HostData 模块
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param cmd CMD 模块引用
   * @param host_euler_topic_name 云台目标欧拉角 Topic
   * @param host_chassis_data_topic_name 底盘目标速度 Topic
   * @param host_fire_topic_name 发射控制 Topic
   */
  HostData(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
           CMD& cmd, const char* host_gimbal_topic_name,
           const char* host_chassis_data_topic_name,
           const char* host_fire_topic_name)
      : cmd_(&cmd),
        host_gimbal_data_tp_(LibXR::Topic::CreateTopic<HostGimbalTarget>(
            host_gimbal_topic_name)),
        host_chassis_data_tp_(LibXR::Topic::CreateTopic<HostChassisTarget>(
            host_chassis_data_topic_name)),
        host_fire_notify_tp_(
            LibXR::Topic::CreateTopic<LauncherCMD>(host_fire_topic_name)) {
    UNUSED(hw);

    auto euler_callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, HostData* host_data,
           const LibXR::ConstRawData& raw_data) {
          HostGimbalTarget t;
          LibXR::Memory::FastCopy(&t, raw_data.addr_, sizeof(t));
          host_data->host_euler_ =
              LibXR::EulerAngle<float>(t.rol, t.pit, t.yaw);
          host_data->host_gyro_ =
              Eigen::Matrix<float, 3, 1>(t.rol_dot, t.pit_dot, t.yaw_dot);
          host_data->host_accl_ =
              Eigen::Matrix<float, 3, 1>(t.rol_ddot, t.pit_ddot, t.yaw_ddot);
          host_data->last_gimbal_time_ = LibXR::Timebase::GetMilliseconds();
          host_data->HostCMD(in_isr);
        },
        this);

    auto chassis_callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, HostData* host_data,
           const LibXR::ConstRawData& raw_data) {
          LibXR::Memory::FastCopy(&host_data->host_chassis_data_,
                                  raw_data.addr_, sizeof(HostChassisTarget));
          host_data->last_chassis_time_ = LibXR::Timebase::GetMilliseconds();
          host_data->HostCMD(in_isr);
        },
        this);

    auto fire_callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, HostData* host_data,
           const LibXR::ConstRawData& raw_data) {
          LibXR::Memory::FastCopy(&host_data->host_fire_notify_, raw_data.addr_,
                                  sizeof(LauncherCMD));
          host_data->last_fire_time_ = LibXR::Timebase::GetMilliseconds();
          host_data->HostCMD(in_isr);
        },
        this);

    host_gimbal_data_tp_.RegisterCallback(euler_callback);
    host_chassis_data_tp_.RegisterCallback(chassis_callback);
    host_fire_notify_tp_.RegisterCallback(fire_callback);

    app.Register(*this);
  }

  /**
   * @brief 汇总并下发 Host 命令
   * @param in_isr 是否在中断上下文（当前未使用）
   */
  void HostCMD(bool in_isr) {
    UNUSED(in_isr);
    auto now = LibXR::Timebase::GetMilliseconds();
    CMD::Data host_cmd = this->BuildHostCMD(now);

    cmd_->FeedAI(host_cmd);
  }

  /**
   * @brief 监控回调
   */
  void OnMonitor() override {
    auto now = LibXR::Timebase::GetMilliseconds();
    const bool CHASSIS_TIMEOUT = !this->IsFresh(last_chassis_time_, now);
    const bool GIMBAL_TIMEOUT = !this->IsFresh(last_gimbal_time_, now);
    const bool FIRE_TIMEOUT = !this->IsFresh(last_fire_time_, now);

    if (!CHASSIS_TIMEOUT && !GIMBAL_TIMEOUT && !FIRE_TIMEOUT) {
      return;
    }

    if (CHASSIS_TIMEOUT) {
      host_chassis_data_ = {};
    }

    if (GIMBAL_TIMEOUT) {
      host_euler_ = LibXR::EulerAngle<float>(0.0f, 0.0f, 0.0f);
      host_gyro_ = {0, 0, 0};
      host_accl_ = {0, 0, 0};
    }

    if (FIRE_TIMEOUT) {
      host_fire_notify_ = {};
    }

    cmd_->FeedAI(this->BuildHostCMD(now));
  }

 private:
  static constexpr uint32_t HOST_DATA_TIMEOUT_MS = 150;

  static bool IsFresh(LibXR::MillisecondTimestamp last_time,
                      LibXR::MillisecondTimestamp now) {
    return static_cast<uint32_t>(last_time) != 0U &&
           (now - last_time).ToMillisecond() <= HOST_DATA_TIMEOUT_MS;
  }

  CMD::Data BuildHostCMD(LibXR::MillisecondTimestamp now) {
    CMD::Data host_cmd = {};
    host_cmd.ctrl_source = CMD::ControlSource::CTRL_SOURCE_AI;

    // 在线状态只由接收时间戳决定，合法的零速度和零角度不能视为离线。
    if (this->IsFresh(last_chassis_time_, now)) {
      host_cmd.chassis.x = host_chassis_data_.vx;
      host_cmd.chassis.y = host_chassis_data_.vy;
      host_cmd.chassis.z = host_chassis_data_.w;
      host_cmd.chassis_online = true;
    }

    if (this->IsFresh(last_gimbal_time_, now)) {
      host_cmd.gimbal.pit = host_euler_.Pitch();
      host_cmd.gimbal.yaw = host_euler_.Yaw();
      host_cmd.gimbal.pit_dot = host_gyro_.y();
      host_cmd.gimbal.pit_ddot = host_accl_.y();
      host_cmd.gimbal.yaw_dot = host_gyro_.z();
      host_cmd.gimbal.yaw_ddot = host_accl_.z();
      host_cmd.gimbal_online = true;
    }

    if (this->IsFresh(last_fire_time_, now)) {
      host_cmd.launcher.isfire = host_fire_notify_.isfire;
    }

    return host_cmd;
  }

  CMD* cmd_;
  HostChassisTarget host_chassis_data_ = {};
  LauncherCMD host_fire_notify_ = {};

  LibXR::EulerAngle<float> host_euler_;
  Eigen::Matrix<float, 3, 1> host_gyro_ = {0, 0, 0};
  Eigen::Matrix<float, 3, 1> host_accl_ = {0, 0, 0};

  LibXR::Topic host_gimbal_data_tp_;
  LibXR::Topic host_chassis_data_tp_;
  LibXR::Topic host_fire_notify_tp_;

  LibXR::MillisecondTimestamp last_chassis_time_ = 0;
  LibXR::MillisecondTimestamp last_gimbal_time_ = 0;
  LibXR::MillisecondTimestamp last_fire_time_ = 0;
};
