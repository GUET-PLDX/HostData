#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - host_gimbal_topic_name: "target_euler"
  - host_chassis_data_topic_name: "host_chassis_data"
  - host_fire_topic_name: "host_fire_notify"
  - task_stack_depth: 1024
  - thread_priority: LibXR::Thread::Priority::MEDIUM
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include "CMD.hpp"
#include "HostDataChassisInput.hpp"
#include "NavLinkProtocol.hpp"
#include "app_framework.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"
#include "message.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

/**
 * @brief 上位机数据接入模块
 * @details 将上位机发送的云台、底盘、发射命令转换为 CMD::Data 并喂给 CMD 模块。
 */
class HostData : public LibXR::Application {
 public:
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
   * @param host_gimbal_topic_name 云台目标欧拉角 Topic
   * @param host_chassis_data_topic_name 底盘目标速度 Topic
   * @param host_fire_topic_name 发射控制 Topic
   * @param task_stack_depth 数据聚合线程栈深度
   * @param thread_priority 数据聚合线程优先级
   */
  HostData(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, CMD& cmd,
      const char* host_gimbal_topic_name,
      const char* host_chassis_data_topic_name,
      const char* host_fire_topic_name, uint32_t task_stack_depth,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM)
      : cmd_(&cmd),
        host_gimbal_data_tp_(LibXR::Topic::CreateTopic<HostGimbalTarget>(
            host_gimbal_topic_name)),
        host_chassis_data_tp_(
            LibXR::Topic::CreateTopic<Pldx::NavLink::ChassisTargetV1>(
                host_chassis_data_topic_name)),
        host_chassis_session_status_tp_(
            LibXR::Topic::CreateTopic<Pldx::HostChassisSession::Status>(
                Pldx::HostChassisSession::STATUS_TOPIC)),
        host_fire_notify_tp_(
            LibXR::Topic::CreateTopic<LauncherCMD>(host_fire_topic_name)) {
    UNUSED(hw);
    app.Register(*this);
    PublishChassisStatus(LibXR::Timebase::GetMilliseconds());
    thread_.Create(this, ThreadFunc, "HostDataThread", task_stack_depth,
                   thread_priority);
  }

  /**
   * @brief 监控回调
   */
  void OnMonitor() override {}

 private:
  static void ThreadFunc(HostData* host_data) {
    LibXR::Topic::ASyncSubscriber<HostGimbalTarget> gimbal_sub(
        host_data->host_gimbal_data_tp_);
    LibXR::Topic::ASyncSubscriber<Pldx::NavLink::ChassisTargetV1> chassis_sub(
        host_data->host_chassis_data_tp_);
    LibXR::Topic::ASyncSubscriber<LauncherCMD> fire_sub(
        host_data->host_fire_notify_tp_);
    gimbal_sub.StartWaiting();
    chassis_sub.StartWaiting();
    fire_sub.StartWaiting();

    LibXR::MillisecondTimestamp last_time = LibXR::Timebase::GetMilliseconds();
    while (true) {
      const auto NOW = LibXR::Timebase::GetMilliseconds();
      bool updated = false;

      if (gimbal_sub.Available()) {
        const auto DATA = gimbal_sub.GetData();
        host_data->ApplyGimbal(DATA, NOW);
        gimbal_sub.StartWaiting();
        updated = true;
      }

      if (chassis_sub.Available()) {
        const bool CHASSIS_ACCEPTED =
            host_data->ApplyChassis(chassis_sub.GetData(), NOW);
        chassis_sub.StartWaiting();
        updated =
            Pldx::HostDataDetail::AccumulateUpdate(updated, CHASSIS_ACCEPTED);
      }

      if (fire_sub.Available()) {
        host_data->host_fire_notify_ = fire_sub.GetData();
        host_data->last_fire_time_ = NOW;
        host_data->fire_received_ = true;
        fire_sub.StartWaiting();
        updated = true;
      }

      const bool FRESHNESS_CHANGED = host_data->FreshnessChanged(NOW);
      if (updated || FRESHNESS_CHANGED) {
        host_data->cmd_->FeedAI(host_data->BuildHostCMD(NOW));
      }

      host_data->thread_.SleepUntil(last_time, 5);
    }
  }

  void ApplyGimbal(const HostGimbalTarget& data,
                   LibXR::MillisecondTimestamp now) {
    host_euler_ = LibXR::EulerAngle<float>(data.rol, data.pit, data.yaw);
    host_gyro_ =
        Eigen::Matrix<float, 3, 1>(data.rol_dot, data.pit_dot, data.yaw_dot);
    host_accl_ =
        Eigen::Matrix<float, 3, 1>(data.rol_ddot, data.pit_ddot, data.yaw_ddot);
    last_gimbal_time_ = now;
    gimbal_received_ = true;
  }

  bool ApplyChassis(const Pldx::NavLink::ChassisTargetV1& data,
                    LibXR::MillisecondTimestamp now) {
    const bool ACCEPTED = chassis_input_.Apply(
        data, now, [this, now] { cmd_->FeedAI(BuildHostCMD(now)); });
    PublishChassisStatus(now);
    return ACCEPTED;
  }

  bool FreshnessChanged(LibXR::MillisecondTimestamp now) {
    auto CHASSIS_STATUS = chassis_input_.SessionStatus(now);
    const bool CHASSIS_FRESH = CHASSIS_STATUS.armed_fresh;
    const bool GIMBAL_FRESH =
        this->IsFresh(gimbal_received_, last_gimbal_time_, now);
    const bool FIRE_FRESH = this->IsFresh(fire_received_, last_fire_time_, now);
    const bool CHANGED = CHASSIS_FRESH != chassis_input_.fresh ||
                         GIMBAL_FRESH != gimbal_fresh_ ||
                         FIRE_FRESH != fire_fresh_;

    if (chassis_input_.fresh && !CHASSIS_FRESH) {
      host_chassis_session_status_tp_.Publish(CHASSIS_STATUS);
    }
    chassis_input_.fresh = CHASSIS_FRESH;
    gimbal_fresh_ = GIMBAL_FRESH;
    fire_fresh_ = FIRE_FRESH;
    return CHANGED;
  }

  static bool IsFresh(bool received, LibXR::MillisecondTimestamp last_time,
                      LibXR::MillisecondTimestamp now) {
    return Pldx::HostDataDetail::IsFresh(received, last_time, now);
  }

  void PublishChassisStatus(LibXR::MillisecondTimestamp now) {
    auto status = chassis_input_.SessionStatus(now);
    host_chassis_session_status_tp_.Publish(status);
  }

  CMD::Data BuildHostCMD(LibXR::MillisecondTimestamp now) {
    CMD::Data host_cmd = {};
    host_cmd.ctrl_source = CMD::ControlSource::CTRL_SOURCE_AI;

    // 在线状态由接收标志和时间戳决定，合法的零值数据不能视为离线。
    if (chassis_input_.IsFreshAt(now)) {
      host_cmd.chassis.x = chassis_input_.target.vx;
      host_cmd.chassis.y = chassis_input_.target.vy;
      host_cmd.chassis.z = chassis_input_.target.wz;
      host_cmd.chassis_online = true;
    }

    if (this->IsFresh(gimbal_received_, last_gimbal_time_, now)) {
      host_cmd.gimbal.pit = host_euler_.Pitch();
      host_cmd.gimbal.yaw = host_euler_.Yaw();
      host_cmd.gimbal.pit_dot = host_gyro_.y();
      host_cmd.gimbal.pit_ddot = host_accl_.y();
      host_cmd.gimbal.yaw_dot = host_gyro_.z();
      host_cmd.gimbal.yaw_ddot = host_accl_.z();
      host_cmd.gimbal_online = true;
    }

    if (this->IsFresh(fire_received_, last_fire_time_, now)) {
      host_cmd.launcher.isfire = host_fire_notify_.isfire;
    }

    return host_cmd;
  }

  CMD* cmd_;
  Pldx::HostDataDetail::ChassisInputState<LibXR::MillisecondTimestamp>
      chassis_input_;
  LauncherCMD host_fire_notify_ = {};

  LibXR::EulerAngle<float> host_euler_;
  Eigen::Matrix<float, 3, 1> host_gyro_ = {0, 0, 0};
  Eigen::Matrix<float, 3, 1> host_accl_ = {0, 0, 0};

  LibXR::Topic host_gimbal_data_tp_;
  LibXR::Topic host_chassis_data_tp_;
  LibXR::Topic host_chassis_session_status_tp_;
  LibXR::Topic host_fire_notify_tp_;

  LibXR::MillisecondTimestamp last_gimbal_time_ = 0;
  LibXR::MillisecondTimestamp last_fire_time_ = 0;

  bool gimbal_received_ = false;
  bool fire_received_ = false;

  bool gimbal_fresh_ = false;
  bool fire_fresh_ = false;

  LibXR::Thread thread_;
};
