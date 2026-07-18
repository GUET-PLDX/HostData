# 简化 AI Yaw 路由与双板构建范围设计

## 1. 目标

将 `Modules/Gimbal/YawLqrEso.hpp` 恢复为只承载 Yaw LQR/ESO 数学控制器的头文件，
删除 `YawRouteState` 及其命令屏障、动作枚举和安全路由状态机。`Gimbal` 直接根据 CMD
当前模式和 AI 云台在线状态选择 LQR 或旧 PID。

工程支持范围同时收缩为双板 Sentry，仅保留：

- `User/RobotConfig/sentry_gimbal.yaml`
- `User/RobotConfig/sentry_chassis.yaml`

本文只定义重构后的行为，不修改 `Modules/CMD` 的现有实现。

## 2. 第一性原理

Yaw LQR/ESO 控制器只需要四类事实：控制参数、连续参考量、物理反馈和控制周期。它不应
理解 CMD 来源、云台模式、电机生命周期或旧 PID。

控制方法选择属于 `Gimbal`：

```text
AI Yaw active = CMD_AUTO_CTRL && AI gimbal online
```

当该条件为真时，Gimbal 使用 `YawLqrEso`；否则使用现有 Yaw PID。除进入 AI 的复位
边沿外，不再维护独立路由状态机。

## 3. 范围

### 3.1 删除

- 删除 `YawLqrEso.hpp` 中完整的 `YawRouteState` 类。
- 删除 `YawRouteState::{Input,Decision,Action}` 及所有状态成员和方法。
- 删除 `Gimbal` 中 `UpdateYawRoute()`、路由动作分支和 `yaw_route_*` 成员。
- 删除 `cmd_sample_seq_` 以及收到 `gimbal_cmd` 时的序号递增。
- 删除来源切换后的新命令屏障；模式切换后接受 CMD 后续自然发布的命令。
- 删除 `ai_yaw_lqr_eso_enable` manifest 字段、构造参数、成员、快照和 YAML 配置。
- 删除仅为路由选择服务的 GM6020 限矩配置校验和 ROTOR 兼容配置校验。
- 删除 `Modules/Gimbal/tests/yaw_route_state_test.cpp`。
- 删除除 `sentry_gimbal.yaml`、`sentry_chassis.yaml` 外的所有
  `User/RobotConfig/*.yaml`，包括带未提交修改的 `sentry.yaml`。

具体删除的配置为：

```text
User/RobotConfig/aerial.yaml
User/RobotConfig/dart.yaml
User/RobotConfig/helm_infantry.yaml
User/RobotConfig/hero.yaml
User/RobotConfig/omni_infantry_3.yaml
User/RobotConfig/omni_infantry_4.yaml
User/RobotConfig/radar.yaml
User/RobotConfig/sentry.yaml
User/RobotConfig/wheel_leg.yaml
```

### 3.2 保留

- 保留 `YawLqrEso::{Config,Reference,Feedback,Output}`。
- 保留 `ValidateConfig()`、`Reset()`、`Calculate()` 和
  `CommitAppliedTorque()`。
- 保留 Gimbal 当前反馈离线触发整机 `RELAX` 的行为。
- 保留 Gimbal 当前 `dt` 有效区间和非法 `dt` 时不运行控制解算的行为。
- 保留 Yaw 电机 `Enable()`、`ClearError()` 和 `Control()` 生命周期分支。
- 保留 CMD 当前遥控失联安全停机、RC/AI 仲裁和 Topic 发布行为。
- 保留历史设计与实施文档；其中旧配置路径仅作为历史记录存在。

## 4. Gimbal 数据流

每个周期在解析 Yaw 目标之前读取 CMD 状态：

```cpp
const bool AI_YAW_ACTIVE =
    cmd_.GetCtrlMode() == CMD::Mode::CMD_AUTO_CTRL &&
    cmd_.GetAIGimbalStatus();
```

`Gimbal` 保存当前 `ai_yaw_active_` 和一个
`yaw_lqr_eso_reset_pending_`。每周期先计算新的 `AI_YAW_ACTIVE`：

- 上升沿将 `yaw_lqr_eso_reset_pending_` 置为 `true`。
- 下降沿调用 `ResetLegacyYawToCurrent()`，使旧 PID 从当前姿态恢复，不追逐进入 AI
  前遗留的目标。
- 随后用 `AI_YAW_ACTIVE` 更新 `ai_yaw_active_`。

进入有效 AI 解算且 `yaw_lqr_eso_reset_pending_` 为 `true` 时调用一次：

```cpp
yaw_lqr_eso_.Reset(euler_.Yaw(), gyro_data_.z(), previous_torque_nm);
```

`Calculate()` 返回有效输出后立即清除 `yaw_lqr_eso_reset_pending_`。这一步不要求电机已经
提交力矩，因此电机未就绪期间的有效计算仍能连续推进状态。退出 AI 不运行 LQR；下一次
重新进入 AI 时再次置位待复位状态。

目标解析规则：

- `AI_YAW_ACTIVE=true`：`SolveAiYaw()` 将
  `cmd_data_.yaw/yaw_dot/yaw_ddot` 直接构造成 `YawLqrEso::Reference`，不先写入
  legacy 目标成员，避免非法 AI 参考污染 `LibXR::CycleValue` 或退出 AI 后的旧 PID 目标。
- `AI_YAW_ACTIVE=false`：完全沿用现有手动、低灵敏度和自动巡逻目标生成逻辑。
- 不等待来源切换后的新 Topic 样本，不再记录命令序号。

控制解算规则：

- `AI_YAW_ACTIVE=true`：调用 `YawLqrEso::Calculate()`。
- `AI_YAW_ACTIVE=false`：调用 `SolveLegacyYaw()`。
- 不再通过五态动作枚举间接分派。

## 5. 运行时行为

| 条件 | Yaw 行为 |
| --- | --- |
| 非自动模式或 AI 云台离线 | 运行旧 Yaw PID |
| 非 AI 到 AI 的第一个可计算周期 | 调用一次 `Reset()`，然后计算 LQR |
| AI 目标、配置或 LQR 输出非法 | 本周期 Yaw 输出 `0.0f` |
| `dt` 非法 | 本周期 Yaw 输出 `0.0f`，不调用 `Calculate()` |
| IMU 或电机反馈离线 | 沿用现有整机 `RELAX` |
| Yaw 电机未使能或正在清错 | 仍计算 LQR，丢弃本周期候选力矩 |
| Yaw 电机可控制 | 发送候选力矩并调用 `CommitAppliedTorque()` |
| 退出 AI | 旧 PID 复位到当前 Yaw 后恢复；再次进入 AI 时重新 `Reset()` |

### 5.1 非法 AI 周期后的复位

AI 目标、配置或输出非法时，Gimbal 将本周期 Yaw 输出置零，并将
`yaw_lqr_eso_reset_pending_` 重新置为 `true`。
下一个可计算的有效 AI 周期先 `Reset()` 再计算，避免继续使用可能已经失真的 observer、
积分、bias 或 slew 状态。

### 5.2 电机未就绪期间的状态推进

当 `Motor::Feedback::state != 1` 但整机反馈仍在线时，Gimbal 仍调用
`YawLqrEso::Calculate()`。电机分支只执行 `Enable()` 或 `ClearError()`，候选力矩不发送、
不调用 `CommitAppliedTorque()`，控制器的其他内部状态允许继续推进。有效计算会清除
`yaw_lqr_eso_reset_pending_`。电机恢复后直接使用后续周期输出，不因电机恢复额外
`Reset()`。

这一行为是明确接受的简化取舍，与原先 `YawRouteState` 的提交门禁不同。

## 6. 配置与构造接口

`Gimbal` manifest 和构造函数移除：

```text
ai_yaw_lqr_eso_enable
```

保留 `YawLqrEso::Config yaw_lqr_eso`。`sentry_gimbal.yaml` 删除总开关，但继续提供经过
当前 GM6020 Sentry 云台配置的完整 `yaw_lqr_eso` 参数。因为工程只保留一个 Gimbal
配置，不再需要跨车型兼容开关。

`sentry_chassis.yaml` 不实例化 Gimbal，不需要新增 LQR 配置。

## 7. 构建和仓库入口

活动构建入口只允许引用两个保留配置：

- `.github/workflows/xrobot_stm32.yml` 的构建与发布矩阵改为
  `sentry_gimbal`、`sentry_chassis`。
- `README.md`、根 `AGENTS.md`、`User/AGENTS.md` 的配置清单和示例改为双板 Sentry。
- `tools/buildgimbal.sh` 和 `tools/buildchassis.sh` 保持为主要便捷入口。
- `tools/build.sh` 的帮助示例不得引用已删除配置。
- 活动测试不得要求已删除配置存在。

历史 `docs/superpowers/specs/` 和 `docs/superpowers/plans/` 不做批量重写。

## 8. 测试设计

### 8.1 算法宿主测试

继续运行 `Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh`，验证纯数学控制器行为。
编译依赖必须仍只有标准库、测试支持和 `YawLqrEso.hpp`。

### 8.2 Gimbal 集成回归

重写现有 AI Yaw 集成回归，至少验证：

- `YawLqrEso.hpp` 不包含 `YawRouteState`。
- `Gimbal.hpp` 不包含 `YawRouteState`、`cmd_sample_seq_` 或
  `ai_yaw_lqr_eso_enable`。
- AI 选择条件精确为 `CMD_AUTO_CTRL && GetAIGimbalStatus()`。
- 非 AI 路径只运行现有 legacy Yaw。
- AI 上升边沿在计算前调用一次 `Reset()`。
- AI 下降边沿将 legacy Yaw PID 和目标复位到当前姿态。
- AI 非法结果置零并重新要求进入复位。
- 电机未就绪时允许先计算；有效计算清除待复位状态，但不调用
  `CommitAppliedTorque()`。
- `CommitAppliedTorque()` 只出现在实际调用 Yaw `Motor::Control()` 之后。
- Pitch 解算和现有整机反馈离线 `RELAX` 行为没有功能变化。

删除状态机行为测试，不用新的状态机测试替代。

### 8.3 配置和引用检查

- 断言 `User/RobotConfig/` 恰好包含两个 YAML 文件。
- 断言活动 CI、构建脚本、README、AGENTS 和活动测试不引用已删除配置。
- 更新配置顺序回归，移除总开关断言，继续验证 `YawLqrEso::Config`、manifest、
  `sentry_gimbal.yaml` 和生成聚合初始化顺序一致。
- `tests/power_control_config_static_regression.sh` 只检查
  `sentry_chassis.yaml` 的 PowerControl 配置，不再枚举已删除车型。
- `tests/chassis_force_control_static_regression.sh` 只检查
  `sentry_chassis.yaml`，删除对单板 Sentry 和 infantry 配置的断言。
- WsProtocol 活动回归只检查 `sentry_gimbal.yaml`；删除对 `sentry.yaml` 的兼容断言，
  同时保留工作树中已存在的 WsProtocol 测试重构意图。
- `tests/remote_failsafe_static_regression.ps1` 保持 CMD 安全停机断言，不因配置收缩而
  改变。

### 8.4 固件验证

分别运行：

```bash
tools/build.sh --skip-format \
  -c User/RobotConfig/sentry_gimbal.yaml \
  -b build/sentry_gimbal

tools/build.sh --skip-format \
  -c User/RobotConfig/sentry_chassis.yaml \
  -b build/sentry_chassis
```

两者必须在 `-Werror` 下成功构建。构建过程生成的 `User/xrobot_main.hpp` 不提交。

## 9. 实施边界

- 不修改 `Modules/CMD`。
- 不修改 `Modules/Motor`、`Modules/RMMotor` 或 `Modules/DMMotor` 的接口。
- 不改变 `YawLqrEso` 数学公式、参数顺序或六个算法功能开关。
- 不新增替代 `YawRouteState` 的类、动作枚举或通用路由抽象。
- 不承诺未实车验证的跟踪性能。
- 不把现有无关工作树修改混入本次提交。

## 10. 完成标准

1. `YawLqrEso.hpp` 只包含数学控制器，不包含云台路由 implementation。
2. Gimbal 的 AI/legacy 选择能从一个直接布尔条件读懂。
3. 所有已确认运行规则都有宿主或结构回归覆盖。
4. `User/RobotConfig/` 只保留双板 Sentry 两个配置。
5. 活动 CI 和文档入口只指向这两个配置。
6. 两个固件配置都能生成并通过完整编译。
