# Sentry AI 自瞄 Yaw LQR+ESO 控制设计

## 1. 决策摘要

本设计在双板 Sentry 工程中移植参考项目的单轴 Yaw LQR+ESO 直接力矩控制器，首个且
唯一启用对象为 `User/RobotConfig/sentry_gimbal.yaml` 的 AI 自瞄 Yaw 分支。

固定边界如下：

- 仅当 `CMD::Mode::CMD_AUTO_CTRL` 且 `CMD::GetAIGimbalStatus()` 为真、Yaw 参考量有限
  且反馈有效时，使用新控制器。
- 手动 Yaw、低灵敏度 Yaw、自动巡逻 Yaw 和 AI 失效后的旧 Yaw 路径保持当前实现。
- 全部 Pitch 目标生成、限位、PID、惯量前馈和重力补偿保持当前实现。
- `Gimbal.hpp` 仍是唯一 XRobot 模块、线程、Topic、模式和电机输出入口；新增
  `YawLqrEso.hpp` 只承载无框架依赖的数学状态和计算。
- 生产代码不引入“初始级/扰动级/完整级”运行时枚举。基础控制链固定存在，附加能力
  由独立布尔开关控制；分阶段只描述测试和启用顺序。
- 本轮根据代码、模型和离线仿真判断实现是否正确，不进行 J-Link 烧录或实车动态验证，
  也不把仿真结果表述为实车性能结论。

本设计比
`docs/superpowers/specs/2026-07-13-gimbal-control-optimization-design.md` 更晚且范围更窄。
二者重叠的 AI Yaw 控制路径以本文为准；本文不撤销旧设计中与 AI Yaw 无关的工作，也
不回退当前 `Modules/Gimbal` 子仓库中的既有改动。实施时以当前工作树行为作为
“保持现状”的基线。

## 2. 参考依据与迁移边界

主要参考文件：

- `.Data/YAW_Auto_Controller-main/yaw_auto_lqr_eso_controller.[ch]`
- `.Data/YAW_Auto_Controller-main/yaw_auto_lqr_eso_controller_usage.md`
- `.Data/YAW_Auto_Controller-main/yaw_auto_lqr_tune.m`
- `.Data/YAW_Auto_Controller-main/单轴云台最优控制器设计.pdf`
- `.Data/【RM2026-为提高自瞄上限的云台控制方案开源】南京理工大学江阴校区-Combat战队-RoboMaster 社区.pdf`

社区 PDF 能支持的结论只有：作者使用连续角度、角速度、角加速度参考驱动 Yaw
LQR+ESO 直接力矩链；展示了标为 `3Hz20deg` 和 `5Hz20deg` 的曲线；参数和限幅必须按
具体云台重新辨识。PDF 没有给出完整方程、增益、采样率、误差指标或 `20deg` 的振幅
定义，也没有证明 GM6020 能复现这些结果。数学迁移以配套源码为准。

配套调参脚本采用 `J=0.039 kg*m^2`、`B=0.30 N*m*s/rad`、`K=[12, 3.4]`、
`1 ms` 和 `7 N*m` 限矩的 DM 场景；C 控制器本身接收调用方 `dt` 和限幅配置。当前
Sentry 使用 GM6020，配置惯量为
`0.03 kg*m^2`，驱动根据 `0.741 N*m/A * 3 A` 得到名义命令峰值
`2.223 N*m`，控制循环约为 `2 ms` 且存在抖动。因此只迁移结构和数学，不直接复制
参考参数。

以下内容明确不属于本轮：

- 视觉包锁存、包间外推、轨迹预测或延迟补偿；
- 修改 `HostData::HostGimbalTarget`、`CMD::GimbalCMD`、SharedTopic 或 DualBoard CAN
  ABI；
- 修改 HostData 的到达时间新鲜度、消息时间戳或包间保持语义；
- 新增第二个 Gimbal 模块、控制线程、控制 Topic 或电机适配模块；
- 在线辨识 `J/B/Kt`、Pitch 角度分段参数或实车调参；
- 修改 RMMotor 的电流到力矩换算；
- 承诺 3 Hz/5 Hz、20 度的实车跟踪能力。

## 3. 文件与所有权结构

生产结构固定为：

```text
Modules/Gimbal/
├── Gimbal.hpp       # 唯一应用、线程、Topic、模式、分支和 Motor::Control 入口
├── YawLqrEso.hpp    # 纯数学控制器及框架无关的单周期路由状态
└── CMakeLists.txt   # 保持现有模块注册；仅在测试接入确有需要时修改
```

`YawLqrEso.hpp` 采用头文件实现，允许包含 C++ 数学与类型头文件，但不得依赖 LibXR、
XRobot、Topic、Thread、Event、Motor、CMD、HardwareContainer 或动态内存。其公开接口由
以下值类型构成：

```cpp
class YawLqrEso {
 public:
  struct Config;
  struct Reference;
  struct Feedback;
  struct Output;

  YawLqrEso() = default;
  static bool ValidateConfig(const Config& config);
  void Reset(float theta_rad, float omega_rad_s,
             float previous_applied_torque_nm);
  Output Calculate(const Config& config, const Reference& reference,
                   const Feedback& feedback, float dt_s);
  void CommitAppliedTorque(float applied_torque_nm);
};
```

同一头文件还提供一个无框架依赖、无电机访问的 `YawRouteState` 测试 seam。它只接收
`route_enable/ai_source/reference_valid/controller_config_valid/feedback_valid/dt_valid/
gimbal_control_enabled/yaw_torque_submission_ready/cmd_sample_seq`，输出单周期动作
`LEGACY_RUN/HOLD_CURRENT/LQR_RUN/ZERO_OUTPUT/RELAX`，并维护 command barrier 与
`rearm_pending`。它提供 `RequestRearm()` 和 `ConfirmLqrCommit()`：只有本周期已越过
RELAX、返回 `LQR_RUN` 且 Yaw `Motor::Control()` 真正提交后，后者才清除 rearm。该动作
枚举是每周期决策结果，不是“初始/扰动/完整”功能阶段，也不拥有任何控制器参数。
`Gimbal` 负责把 CMD、当前模式和反馈映射为输入，再执行动作。

`Config` 保存模型、增益、限幅和独立开关；`Reference` 保存连续参考
`theta/omega/alpha`；`Feedback` 保存惯性 Yaw、IMU gyro-z 和 GM6020 电流反算力矩；
`Output` 保存最终力矩、各分项、ESO 状态、限幅标志和 `valid`。bias 关闭时，测量力矩
不参与反馈有效性判定。类不抛异常、不分配内存，所有状态都可通过 `Reset()` 确定性
复位。

`Calculate()` 只产生候选输出，不假定该输出已送达电机。`Gimbal` 只有在本周期确实调用
Yaw 电机的 `Motor::Control()` 后才调用 `CommitAppliedTorque()`；ESO 和 slew 下一周期
都使用最后一次已提交命令。执行 `Enable()`、`ClearError()` 或 `Relax()` 的周期不得把
候选力矩提交为已施加力矩。

内部必须分别保存 `tau_cmd_last_applied` 和 `tau_slew_anchor`：前者无论 slew 是否开启都
供 ESO 使用，后者只服务斜率限制并受 `torque_slew_enable` 的边沿复位。不得为了省一个
float 合并两种状态，否则关闭 slew 会同时破坏 ESO 输入历史。

`Gimbal` 拥有一个可观察配置、一个控制器和一个最近输出快照。每周期把本周期的
`Config` 只读快照按 const 引用传入 `Calculate()`，所以通过调试器修改六个开关时，
数学类能在下一周期检测边沿并执行“关闭即清状态”；控制器不得只在构造时复制一次
配置。输出快照仅供调试器和主机测试观察，不发布新 Topic，也不参与其他模块的控制。

每周期在路由校验前先把 master route 和可观察配置复制为本周期只读快照；
`ValidateConfig()`、`gm6020_limit_valid`、开关边沿和 `Calculate()` 必须使用同一快照。
调试器多字段写入期间
若形成暂时非法快照，本周期 `HOLD_CURRENT`，不得出现“用旧边界通过校验、用新边界
计算”的撕裂行为。

`Gimbal` 还必须持有 `last_submitted_yaw_torque_nm` 和有效标志。只有 Yaw
`motor_yaw_->Control()` 真正被调用后，才同时更新该账本并调用
`YawLqrEso::CommitAppliedTorque()`；执行 `Enable/ClearError/Relax/Disable` 时账本归零且
标为未提交，同时调用 `YawRouteState::RequestRearm()`，禁止本周期推进 ESO、LQI、bias
或 slew 状态。算法输出无效而回退旧 PID 时，如果旧 PID 命令确实提交，也按其最终命令
更新账本。进入 AI 只能读取该账本，不能读取每周期开头被清零的 `yaw_output_` 或尚未
提交的候选输出。

### 3.1 LibXR 与模块代码风格

实施必须优先复用 LibXR 和现有模块设施：

- `Gimbal.hpp` 继续使用现有 `LibXR::Application`、`Thread`、`Topic`、`Event`、
  `Timebase`、`CycleValue`、`ErrorCode` 和 legacy `PID`，不得为调度、时间戳、角度周期
  值、事件或错误码另造基础设施。
- 不修改 `Middlewares/Third_Party/LibXR`，也不为了本任务引入另一套框架封装。
- `YawLqrEso.hpp` 的框架无关是有意的最小例外，目的是让数学与路由状态能在宿主机独立
  测试；其中只使用标准数学/类型工具，不复制 LibXR 的线程、Topic 或应用能力。
- `Gimbal.hpp` 与 `YawLqrEso.hpp` 的职责划分效仿 `PowerControl.hpp + RLS.hpp`：应用
  生命周期和输出留在主模块，辅助头只保存确定性算法状态。

代码必须遵守当前 `Modules/` 风格和仓库命名规则：变量为 `lower_case`，类私有成员带
尾下划线，类/结构/枚举为 `CamelCase`，方法为 `CamelCase`，全部常量为
`UPPER_CASE`；公开配置字段按现有 `Param` 结构使用 `lower_case`。manifest、构造参数和
YAML 保持一一对应；注释只解释非直观的状态切换和安全边界。修改后使用工程指定的
clang-format 版本格式化 `Modules/`，不得手工调整格式化后的 include 顺序，也不得加入
诊断抑制 pragma。

## 4. 路由与配置兼容

### 4.1 AI Yaw 路由条件

当前 AI 判定不是 `GimbalEvent`，而是 CMD 仲裁结果。新路径的请求条件为：

```text
ai_source = cmd.GetCtrlMode() == CMD_AUTO_CTRL && cmd.GetAIGimbalStatus()
gimbal_control_enabled = current_mode != SET_MODE_RELAX
yaw_torque_submission_ready = motor_yaw_feedback.state == 1
cmd_source_coherent = !ai_yaw_lqr_eso_enable ||
                      cmd_sample_seq > source_edge_cmd_sample_seq
ai_yaw_reference_valid = finite(yaw, yaw_dot, yaw_ddot)
ai_yaw_config_valid = controller_config_valid && gm6020_limit_valid
ai_yaw_lqr_selected = ai_yaw_lqr_eso_enable && ai_source &&
                      cmd_source_coherent &&
                      ai_yaw_reference_valid && ai_yaw_config_valid
ai_yaw_step_valid = ai_yaw_lqr_selected && feedback_valid && dt_valid &&
                    gimbal_control_enabled && yaw_torque_submission_ready
```

CMD 的 mode/AI-online 状态与 `gimbal_cmd` Topic 不是原子快照，且 `SetCtrlMode()` 本身不
重新发布命令。`Gimbal` 因此为收到的 `gimbal_cmd` 维护本地单调
`cmd_sample_seq`。`route_enable` 从 false 变 true，或 `ai_source` 上升/下降时，都记录
当前序号并进入 command barrier；必须再收到一份序号更大的 Topic 样本，才能把该样本
按新来源语义解释。
屏障期间 Yaw 目标锁在当前角并使用重置后的旧 PID 保持，不能把旧 RC 速率当作 AI
绝对角，也不能把旧 AI 角度当作 RC 速率。Pitch 仍沿用当前 Topic 消费行为。

进入和退出边沿由 `ai_yaw_lqr_selected` 决定。已选择但 `dt` 非法时执行零输出和待重置，
反馈非法时执行既有 RELAX；二者都不能临时改走旧 PID。参考或配置非法时按第 6.3 节
退出新路径并由旧 PID 保持当前角。

当 master route 已启用且 `ai_source=true`，但参考或控制器配置非法时，
`YawRouteState` 返回 `HOLD_CURRENT`，不得把同一 AI 目标交给 legacy PID 继续追踪。只有
master route 本身关闭时，有限输入才按兼容要求完整走旧 AI Yaw PID。

`gimbal_control_enabled=false` 时返回 `RELAX`；模式允许但
`yaw_torque_submission_ready=false` 时返回 `HOLD_CURRENT` 并保持 rearm，由现有电机分支
执行 `Enable()` 或 `ClearError()`。只有电机 `state==1`、本周期将进入
`Motor::Control()` 时才允许 `LQR_RUN` 推进算法状态。

`YawRouteState` 的 `rearm_pending` 与 source 边沿分离。进入 AI、非法 `dt`、RELAX、
反馈失效或算法状态复位都会置位；下一次 `ai_yaw_step_valid` 时先按第 6.2 节重装，再在
同一周期计算基础 LQR。若本周期只执行 `Enable/ClearError/Relax` 而没有提交 Yaw 控制
命令，rearm 保持置位；恢复不依赖再次出现 source 上升沿，也不会在 RELAX 中被提前
消费。

`ai_yaw_lqr_eso_enable` 是跨车型兼容门禁，不是三阶段状态，也不控制任何补偿项。它在
`sentry_gimbal.yaml` 中固定为 `true`，构造默认值为 `false`，从而使未显式迁移的
`sentry`、`hero`、`aerial` 和 infantry 配置继续使用旧 AI Yaw PID。目标 Sentry 的
基础 LQR 路径没有运行时阶段选择。

该 `true` 配置是按用户要求生成的实验构建默认值，用于后续受控台架观察，不代表已通过
实车验证或可直接参加比赛。首次上电仍需机械隔离、急停和低幅目标等外部措施；这些
实机步骤不属于本轮离线验收。

### 4.2 构造参数

在 `Gimbal` 构造参数末尾追加带默认值的 `ai_yaw_lqr_eso_enable` 和
`YawLqrEso::Config yaw_lqr_eso`，并同步 manifest。现有 YAML 不填写时必须仍能生成和
编译；只有 `sentry_gimbal.yaml` 显式启用并覆盖配置。不得手工编辑生成的
`User/xrobot_main.hpp`。

XRobot 会按嵌套 mapping 的出现顺序生成聚合初始化，字段名不会进入 C++。因此
`Config` 声明、Gimbal manifest 和 `sentry_gimbal.yaml` 必须逐项使用以下唯一顺序：

```cpp
struct Config {
  float j_kg_m2;
  float b_nms_rad;
  float k_theta;
  float k_omega;
  float k_i;
  float theta_integral_limit_rad_s;
  float tau_coulomb_nm;
  float coulomb_smooth_rad_s;
  float eso_bandwidth_rad_s;
  float eso_comp_gain;
  float eso_comp_limit_nm;
  float eso_omega_gate_rad_s;
  float eso_alpha_gate_rad_s2;
  float tau_bias_ki;
  float tau_bias_limit_nm;
  float tau_meas_lpf_alpha;
  float theta_deadband_rad;
  float torque_soft_limit_nm;
  float torque_min_nm;
  float torque_max_nm;
  float torque_slew_rate_nm_s;
  bool eso_enable;
  bool eso_comp_enable;
  bool coulomb_enable;
  bool lqi_enable;
  bool torque_bias_enable;
  bool torque_slew_enable;
};
```

测试必须比较三处字段顺序，并检查生成代码中的聚合值序列；仅“能够编译”不足以发现
多个 float 错位。

算法开关固定为六个：

| 开关 | 含义 | 关闭行为 |
|---|---|---|
| `eso_enable` | 运行三阶 ESO | 清空并重新对齐 observer 状态 |
| `eso_comp_enable` | 将 ESO 扰动估计接入力矩 | observer 可继续仅观测，补偿输出为零 |
| `coulomb_enable` | 加入平滑 Coulomb 前馈 | 前馈项为零，无持久状态 |
| `lqi_enable` | 加入角误差积分反馈 | 每周期清空积分状态 |
| `torque_bias_enable` | 根据测量力矩误差积分 bias | 清空 bias 和测量滤波初始化状态 |
| `torque_slew_enable` | 最终力矩斜率限制 | 清除 slew 锚点；上一最终命令仍供 ESO 使用 |

六个开关是本项目为可控启用而增加的接口。参考头文件只直接提供
`eso_enable`、`eso_comp_enable` 和 `torque_slew_enable`；Coulomb、LQI 和 bias 在参考
实现中通过零参数隐式关闭，本文将其改为显式开关。

不增加 `INITIAL/DISTURBANCE/FULL` 等阶段字段，也不让某个开关隐式开启另一个开关。
唯一依赖是 ESO 补偿必须同时满足 `eso_enable`、observer ready 和门控条件。

### 4.3 Sentry 首次配置

以下数值是代码推测的安全起点，不是辨识结果：

| 参数 | 初值 | 说明 |
|---|---:|---|
| `j_kg_m2` | `0.03` | 复用当前 `j_yaw` |
| `b_nms_rad` | `0.0` | 当前 `yaw_k` 为零 |
| `k_theta` | `1.0` | 与当前角度 P 增益同量级 |
| `k_omega` | `1.0` | 与当前速度 P 增益同量级 |
| `k_i` | `0.2` | 关闭状态下不生效；只作后续低增益候选 |
| `theta_integral_limit_rad_s` | `0.5` | LQI 力矩贡献最多 `0.1 N*m` |
| `tau_coulomb_nm` | `0.05` | 关闭状态下不生效 |
| `coulomb_smooth_rad_s` | `0.2` | 避免零速符号跳变 |
| `eso_bandwidth_rad_s` | `30.0` | 首次仅观测 |
| `eso_comp_gain` | `1.0` | 补偿关闭时不生效 |
| `eso_comp_limit_nm` | `0.3` | 限制 ESO 最大附加力矩 |
| `eso_omega_gate_rad_s` | `5.0` | 低速门控候选 |
| `eso_alpha_gate_rad_s2` | `50.0` | 低动态门控候选 |
| `tau_bias_ki` | `0.5` | 关闭状态下不生效 |
| `tau_bias_limit_nm` | `0.15` | 限制测量 bias |
| `tau_meas_lpf_alpha` | `0.1` | GM6020 电流反算力矩低通候选 |
| `theta_deadband_rad` | `0.0` | 初始不引入死区 |
| `torque_soft_limit_nm` | `2.0` | 在驱动名义峰值前保留余量 |
| `torque_min_nm` | `-2.223` | GM6020 名义命令下限 |
| `torque_max_nm` | `2.223` | GM6020 名义命令上限 |
| `torque_slew_rate_nm_s` | `1000.0` | 2 ms 时最多变化约 `2 N*m` |

目标 Sentry 的 `gm6020_limit_valid` 精确定义为
`-2.223 <= torque_min_nm < 0 < torque_max_nm <= 2.223`；只有为真时才允许选中 LQR
路径。纯数学类保留参考代码“无有效区间则不做
该级 clamp”的通用语义，但该语义不能用于启用 GM6020 的目标配置；不满足时必须回退旧
PID，而不是仅依赖 RMMotor 末级 clamp。

首次开关值为：

```text
eso_enable          = true
eso_comp_enable     = false
coulomb_enable      = false
lqi_enable          = false
torque_bias_enable  = false
torque_slew_enable  = true
```

`K=[1,1]` 只是把当前双 P 增益映射为保守的 LQR 形式全状态反馈起点，并非严格等效，
也不是由当前对象的 Riccati 方程求得；当前实现还包含角度环输出差分产生的惯量项。
`[3.8,1.1]` 同样是经验候选。离线还比较参考 `[12,3.4]`，但首次实验配置不因仿真
排名自动改成高增益候选。

现有 `j_yaw` 和 `yaw_k` 继续只服务旧 Yaw PID 路径；新配置中的 `j_kg_m2` 和
`b_nms_rad` 只服务 AI LQR+ESO。Sentry 首值有意保持两边数值一致，但实施不得为了消除
重复而改写手动/巡逻路径的参数接口。

## 5. 控制数学

### 5.1 坐标、角度展开和误差

所有量使用 SI 单位：rad、rad/s、rad/s^2、N*m、s。反馈沿用当前惯性系
`gimbal_euler.Yaw()` 和 `gimbal_gyro.z()`，不改符号或坐标定义。

当前 Euler Yaw 会跨周期边界，而 ESO 不能直接观察跳变角度。因此数学类内部维护连续
测量角：

```text
delta_theta = wrap_pi(theta_meas_raw - theta_meas_raw_last)
theta_unwrapped += delta_theta
e_theta = deadband(wrap_pi(theta_meas_raw - theta_ref_raw))
e_omega = omega_meas - omega_ref
```

`Reset()` 以当前原始 Yaw 初始化展开状态和 `z1`。这保持当前
`LibXR::CycleValue` 的最短角误差语义，同时避免跨 `0/2pi` 或 `+/-pi` 时冲击 ESO。
控制器假设单周期真实转角小于 pi；以当前约 2 ms 周期和 GM6020 能力，该条件有充分
余量。

### 5.2 基础 LQR 与前馈

对象模型为：

```text
J * theta_ddot + B * theta_dot = tau + d
```

基础控制链始终为：

```text
tau_ff = J * alpha_ref + B * omega_ref
tau_fb = -K_theta * e_theta - K_omega * e_omega
tau_base = tau_ff + tau_fb
```

若 `coulomb_enable`：

```text
tau_coulomb = tau_c * tanh(omega_ref / omega_s)
```

若 `lqi_enable`：

```text
integral = clamp(integral + e_theta * dt, -integral_limit, integral_limit)
tau_lqi = -K_i * integral
```

最终基础项为：

```text
tau_lqr = tau_base + tau_coulomb + tau_lqi
```

`J*alpha_ref` 只使用 HostData/CMD 已提供的 `yaw_ddot`，不再对 LQR 角度反馈输出做差分，
从而避免把旧级联 PID 的角度环导数项带入新控制器。`B*omega_ref` 与参考实现一致；本轮
不增加底盘角速度或角加速度到 LQR/ESO 模型。旧 Yaw PID 分支现有的
`rotor_ff_enabled` 行为保持不变。

因此首次配置必须保持 `B=0`、Coulomb 关闭和 ESO compensation 关闭。ROTOR 时惯性
`gyro-z` 接近零并不代表 GM6020 相对底盘低速，现有 ESO 速度门控可能误判；在未来
补齐相对速度模型或获得正反转数据前，不允许在 ROTOR 场景开启这三项。observer-only
仍可运行，因为其状态不接入力矩。

### 5.3 三阶 ESO

定义：

```text
b0 = 1 / J
beta1 = 3 * w0
beta2 = 3 * w0^2
beta3 = w0^3
observer_error = theta_unwrapped - z1
```

使用前向 Euler 和上一周期最终实际下发命令 `tau_cmd_last`：

```text
z1 += dt * (z2 + beta1 * observer_error)
z2 += dt * (-(B/J) * z2 + b0 * tau_cmd_last + z3 +
            beta2 * observer_error)
z3 += dt * (beta3 * observer_error)
```

三个导数必须从同一份更新前状态计算，再一次性提交新 `z1/z2/z3`；不得让代码书写顺序
形成半隐式更新。

原始补偿为：

```text
tau_eso_raw = clamp(-eso_comp_gain * z3 / b0,
                    -eso_comp_limit, eso_comp_limit)
```

只有以下条件全部满足才令 `tau_eso_active=tau_eso_raw`：

```text
eso_enable && eso_comp_enable && observer_ready &&
abs(omega_meas) <= eso_omega_gate &&
abs(alpha_ref) <= eso_alpha_gate
```

门限小于等于零时沿用参考语义，表示对应门控不限制；Sentry 配置在开启补偿时必须使用
正门限。首次配置 `eso_enable=true`、`eso_comp_enable=false`，所以 observer 更新并输出
诊断，绝不改变控制力矩。

observer 是与基础力矩隔离的旁路：若 `z1/z2/z3` 更新产生非有限值，立即丢弃候选状态、
重置 observer 并令本周期 ESO 补偿为零。只要基础 LQR 输入和输出仍有限，就继续输出
基础 LQR；observer-only 的异常不得触发旧 PID 回退。即使补偿开关为真，observer 异常
也只撤除补偿并等待重新 ready，不能让非有限值传播到主力矩路径。

### 5.4 测量力矩 bias

GM6020 的 `Motor::Feedback::torque` 是 CAN 电流按力矩常数反算的估计，不是真实轴端
力矩。该值只在 `torque_bias_enable=true` 时使用：

```text
tau_meas_filt += alpha * (tau_meas - tau_meas_filt)
tau_without_bias = tau_lqr + tau_eso_active
tau_bias += tau_bias_ki * (tau_without_bias - tau_meas_filt) * dt
tau_bias = clamp(tau_bias, -tau_bias_limit, tau_bias_limit)
tau_pre_limit = tau_without_bias + tau_bias
```

首次启用 bias 时以当前有限测量值初始化低通，避免从零产生假误差。实现保留参考公式，
不把该项描述为真实负载力矩估计，也不默认开启。

为保持完整移植，本轮 LQI 和 bias 只使用状态幅值 clamp，不额外发明饱和反算或条件
积分；软/硬/slew 激活时它们仍可能积分到各自上限。这是已知限制，也是二者默认关闭、
必须分别验证后才能启用的原因。bias 启用时还要求 `tau_meas_lpf_alpha > 0`。

### 5.5 输出约束顺序

顺序固定为：

```text
LQR + J*alpha + B*omega + 可选 Coulomb/LQI
  -> 可选 ESO 补偿
  -> 可选测量力矩 bias
  -> 软限幅
  -> 硬限幅
  -> 可选斜率限制
  -> 有限值终检
  -> Motor::Control(MODE_TORQUE)
```

规则如下：

- `torque_soft_limit_nm > 0` 时，软限幅按参考代码执行对称 clamp。
- `torque_min_nm < torque_max_nm` 时，硬限幅自动生效；它没有独立开关。
- slew 的单周期最大变化量为 `torque_slew_rate_nm_s * dt`。
- 每周期在执行 slew 前，先把 `tau_slew_anchor` 投影到当前软/硬限幅有效区间。这样通过
  调试器收紧边界时，anchor 和已限幅目标都位于新边界内，slew 不会把输出重新拉出
  硬限幅；边界突变时允许为满足硬安全而跳过变化率约束。
- ESO 下一周期输入使用经过软限幅、硬限幅和 slew 后，本周期真正传给
  `Motor::Control()` 的最终 Yaw 命令。
- RMMotor 内部限流仍是最后兜底，不代替算法硬限幅。
- RELAX、反馈失效和非法 `dt` 的零力矩安全动作绕过 slew，不能缓慢撤除。

## 6. Gimbal 集成与状态转换

### 6.1 单周期数据流

现有线程顺序保持：

```text
Update -> ParseCMD -> Control -> Sleep(2)
```

`ParseCMD()` 继续生成全部 Pitch 目标和非 AI Yaw 目标。AI 在线时仍直接使用 CMD 中的
`yaw/yaw_dot/yaw_ddot`；控制器不判断视觉包是否更新，也不外推参考。Yaw 三元组必须先
完成有限值检查，再写入 `target_yaw_cmd_` 及其导数，非法值不得进入
`LibXR::CycleValue`。Yaw 非法不改变同一包中 Pitch 的既有处理。

AI source、Yaw 参考有效性和进入/退出边沿必须在 `ParseCMD()` 开头、任何目标生成之前
处理。此时本周期 `Update()` 已完成，当前反馈和上一周期已提交力矩都可用。不得等到
`Control()` 或 `Solve()` 后再重置目标，否则手动/巡逻分支已经错过本周期的目标生成。

`Solve()` 中先执行当前 Pitch 代码，再按以下优先级选择 Yaw 行为：

```text
LQR_RUN      -> YawLqrEso::Calculate()
ZERO_OUTPUT  -> Yaw 零输出并置 rearm
RELAX        -> 既有 SET_MODE_RELAX
HOLD_CURRENT -> 重置后的旧 PID 保持当前角，不消费不一致/非法 Yaw 命令
LEGACY_RUN   -> 当前 Yaw 角度 PID + 速度 PID + 既有前馈
```

旧 Yaw 路径的 PID 参数、目标差分、`rotor_ff_enabled`、输出语义和电机接口不改。

### 6.2 进入 AI Yaw

AI source 上升沿或 `YawRouteState::rearm_pending` 在合法周期的重装按以下顺序处理：

1. 保存上一周期真正送入 `Motor::Control()` 的最终 Yaw 命令，作为 slew 初始锚点。
2. 以当前展开 Yaw 和 gyro-z 初始化 ESO `z1/z2`。
3. 清零 `z3`、LQI 积分、bias 和测量滤波初始化标志。
4. 标记本周期为 observer fresh；本周期不推进 ESO，下一合法周期开始使用上一最终命令
   更新 observer。
5. 本周期直接计算基础 LQR 输出，并由 slew 从旧 PID 命令平滑过渡，不人为插入零力矩
   周期。
6. 只有 Yaw `Motor::Control()` 被调用并完成账本/`CommitAppliedTorque()` 更新后，调用
   `YawRouteState::ConfirmLqrCommit()` 清除 rearm；否则下一周期重新执行以上初始化。

参考代码首次收到有效反馈时只同步状态并输出零；上述“旧 PID 锚点 + 同周期基础 LQR”
是本项目为避免人为零力矩空窗而采用的切换适配，不是参考实现原样行为，必须由专门的
进入/退出测试锁定。

这里的“实际命令”只表示软件能够确认已调用 `Motor::Control()` 的最终命令，不声称是
真实轴端力矩。若上一周期只执行了 `Enable()`、`ClearError()` 或 `Relax()`，锚点为零。

### 6.3 退出 AI Yaw

AI 来源消失、AI freshness 变假、Yaw 参考非法或控制器配置非法时：

1. 立即复位 `YawLqrEso` 的 observer、积分、bias 和 slew 状态。
2. 重置 Yaw 角度环和速度环 PID，并清零其 feedforward/导数历史。
3. 把 Yaw 目标设为当前 Yaw；command barrier 未满足时由重置后的旧 PID 保持该角，收到
   一份切换后的新 `gimbal_cmd` 样本后，旧手动/巡逻分支再从该目标继续生成命令。
4. Pitch 不复位、不改目标，继续当前路径。

这样退出不会把最后一个 AI 目标或算法状态带入手动/巡逻 Yaw，也不会更改现有 CMD
仲裁和 HostData 超时。上述动作发生在 `ParseCMD()` 开头；屏障会至少等待一个切换后的
命令发布，无新样本时持续保持当前角。这是有意安全行为，避免把切换前样本按切换后
语义解释。

### 6.4 开关状态

配置开关即使通过调试器在运行时改变，也必须满足“关闭即清状态”：

- 关闭 observer：清除 observer ready，下一次开启从当前反馈重新初始化。
- 关闭 LQI：积分立即归零。
- 关闭 bias：bias、滤波值和滤波 ready 立即归零。
- 关闭 slew：清除 slew ready；再次开启时从最近最终命令初始化。
- Coulomb 和 ESO compensation 没有独立持久状态；关闭后本周期贡献立即为零。

## 7. 非法输入与失效安全

当前 Gimbal 的 IMU、电机和 `dt` 保护继续作为外层安全边界。新增规则为：

- `dt` 必须继续满足当前 `0.5 ms < dt <= 20 ms`。非法时不使用参考实现的
  “替换为 1 ms”策略；本周期 Yaw 输出零、控制器复位并等待下一合法周期重新初始化。
- IMU 或任一电机反馈失效时沿用当前 `SET_MODE_RELAX`，并复位控制器。RELAX 直接调用
  `Motor::Relax()`，不走 slew。
- `yaw/yaw_dot/yaw_ddot` 任一非有限时，只退出 AI LQR；Yaw 目标锁在当前角并由旧 PID
  保持。Pitch 和 CMD 状态不因此改写。
- 配置字段、基础反馈、基础 LQR/bias/限幅中间量或最终输出任一非有限，
  `Output.valid=false`，本周期禁止该值进入 `Motor::Control()`。在传感器和 `dt` 仍有效
  时，Yaw 回退到重置后的旧 PID 保持当前角；否则使用既有零输出或 RELAX。ESO 状态
  异常按第 5.3 节隔离处理，不使基础 `Output` 失效。
- `J` 必须有限且大于 `1e-6`；全部增益、限幅和门限必须有限。配置不合法时不得静默
  替换成参考默认值。
- 基础配置还要求 `B/K_theta/K_omega >= 0`。对应功能开启时，积分限值、ESO 带宽、
  补偿限值、Coulomb 平滑速度、bias 限值和 slew rate 必须满足其公式所需的正值，
  `K_i/eso_comp_gain/tau_coulomb/tau_bias_ki` 必须非负，
  `tau_meas_lpf_alpha` 必须位于 `(0,1]`。硬限幅仍只按
  `torque_min < torque_max` 自动启用，未启用不等于配置错误。
- `Gimbal` 在构造 Yaw `Motor::MotorCmd` 前执行最后一次 `std::isfinite` 检查，保证
  新控制路径的 NaN/Inf 不会到达电机接口；Pitch 行为不在本轮扩展范围内。

## 8. 离线验证

### 8.1 框架无关测试

为 `YawLqrEso.hpp` 增加主机侧确定性测试，至少覆盖：

- 零误差、正负角度误差和正负速度误差的反馈符号；
- `J*alpha`、`B*omega`、Coulomb 和 LQI 各分项及积分限幅；
- 跨 `0/2pi` 和 `+/-pi` 的测量展开与最短角误差；
- ESO 初始化、observer-only、上一最终命令输入、补偿符号、补偿限幅和双门控；
- observer 非有限状态隔离，以及 `Calculate()` 未提交时不得推进 applied-torque 锚点；
- bias 低通、积分、限幅和开关复位；
- 软限幅、硬限幅、slew 的固定顺序和 active 标志，以及运行时收紧边界时的 anchor
  投影和硬安全优先行为；
- 六个开关的独立性及关闭清状态；
- 非法 `dt`、非法 `J`、NaN/Inf 输入和内部溢出的无输出行为；
- 进入/退出时以前一 PID 命令初始化 slew，RELAX/失效绕过 slew。
- 非法 `dt` 置 rearm 后，在下一合法周期无需 source 重入即可重新初始化。

`YawRouteState` 的可执行测试另覆盖：AUTO/AI 上升和下降、切换后没有新 Topic 样本、
RC/AI 旧样本屏障、非法参考恢复、非法 `dt` rearm、反馈失效 RELAX，以及 route 关闭时
对有限输入始终选择 legacy。还要覆盖 GM6020 硬限幅配置非法时 `HOLD_CURRENT`、RELAX
中不消费 rearm、Yaw 电机处于 Enable/ClearError 分支时不推进算法，以及未收到实际
LQR commit 时反复保持 rearm。纯算法测试不冒充 Gimbal/Motor 集成测试。

测试必须能在宿主机独立编译，不需要 HAL、FreeRTOS、LibXR 或板卡。

### 8.2 Gimbal 集成回归

在现有 `Modules/Gimbal/tests/gimbal_core_static_regression.sh` 基础上增加结构回归，检查：

- source/barrier 决策位于 `ParseCMD()` 任何 Yaw 目标写入之前；
- 非法 Yaw 三元组不会写入 `LibXR::CycleValue`；
- `CommitAppliedTorque()` 只在 Yaw `Motor::Control()` 调用之后出现，其他电机动作清空
  Gimbal 提交账本并请求 rearm；
- LQR 分支只由 `YawRouteState::LQR_RUN` 进入；
- Pitch 解算块和 legacy Yaw 公式相对实施前基线没有功能改动；
- manifest、`Config`、目标 YAML 和生成聚合初始化顺序一致。

这部分结合 `YawRouteState` 可执行测试、源结构回归和真实固件编译证明集成边界；不把
正则脚本单独描述为行为测试。若实施时无法用稳定的结构断言证明 Commit 顺序，必须把
Yaw 电机提交封装成 `Gimbal.hpp` 内唯一私有方法，使“Control 成功调用后才记账”由代码
结构保证。

### 8.3 二阶对象仿真

仿真对象使用：

```text
J * theta_ddot + B * theta_dot = tau_cmd + disturbance
```

基准周期固定为 `2 ms`；抖动测试循环使用字面序列
`{1.5, 2.0, 2.5, 2.0} ms`，不使用平台相关随机数。比较四条控制路径：

- 当前 AI Yaw 级联 PID 公式；
- LQR 形式状态反馈 `K=[1,1]`；
- 经验状态反馈候选 `K=[3.8,1.1]`；
- 参考 `K=[12,3.4]`。

所有正弦测试都由解析式生成一致的 `theta/omega/alpha`，并明确“角度”为峰值而非借用
参考 PDF 的 `20deg` 名称。矩阵为：

| 类别 | 工况 | 目的 |
|---|---|---|
| 基础 | 静止保持、正负平滑阶跃 | 符号、超调、稳定时间、切换 |
| 低频 | `1 Hz, +/-10 deg` | 基线跟踪与低速补偿 |
| 名义 | `3 Hz, +/-5 deg`、`+/-8 deg` | 中高频可行区 |
| 名义 | `5 Hz, +/-2 deg`、`+/-3 deg` | 高频小幅可行区 |
| 边界 | `3 Hz, +/-10 deg` | 饱和与力矩余量 |
| 过载 | `5 Hz, +/-10 deg` 后回到保持 | 限幅、无发散和恢复，不考核跟踪精度 |

平滑阶跃固定为从 0 到 `+/-5 deg` 的五次 minimum-jerk 轨迹，公式为
`s(r)=10r^3-15r^4+6r^5`、上升时间 `0.2 s`，随后保持 `5 s`。正弦先预热 2 个周期，
再统计 10 个完整周期；过载运行 `3.25` 个周期，在正峰值处切换到一条 `0.2 s` 的
五次多项式，该多项式匹配切换点 `theta/omega/alpha` 并以零 `theta/omega/alpha` 结束，
随后观察 `5 s`。

模型失配扫描固定控制器配置 `J=0.03、B=0`，只改变对象：
`J={0.021,0.030,0.039}`、`B={0.0,0.1,0.2,0.3}`、
`disturbance={-0.2,0.0,+0.2} N*m`。名义对象运行全部轨迹；完整失配网格运行
`3 Hz +/-8 deg` 和 `5 Hz +/-3 deg`，并分别使用固定周期和上述抖动序列。没有实测链路
日志时，不虚构视觉延迟、丢包或噪声分布；这些只在未来有数据后追加。

记录指标：角度/速度 RMSE、p95、超调、稳定时间、基频相位、峰值/RMS 力矩、软/硬/
slew 限制占比、ESO 估计误差和过载恢复时间。基频相位通过统计窗口内的最小二乘正弦
拟合计算，仅在软/硬限幅总占比小于 `1%` 时报告；其他工况标为 saturated，不给出易
误导的相位值。

### 8.4 物理边界解释

对本文定义的正弦 `theta=A*sin(2*pi*f*t)`，仅惯量前馈峰值为：

```text
tau_peak = J * A * (2*pi*f)^2
```

按 `J=0.03` 和 20 度峰峰值，即 `A=10 deg`：

- 3 Hz 需要约 `1.86 N*m` 峰值、`1.32 N*m` RMS，已接近或超过 GM6020 连续能力，
  只能视为短时边界工况。按首次 `2.0 N*m` 软限幅，纯惯性理论上限约为
  `+/-10.75 deg`，所以 `+/-10 deg` 只剩约 `0.14 N*m` 峰值余量给反馈、阻尼和扰动。
- 5 Hz 需要约 `5.17 N*m` 峰值，超过 `2.223 N*m` 名义命令峰值，不可能无饱和跟踪。
- 忽略阻尼、扰动和反馈修正，5 Hz 在 `2.223 N*m` 名义命令峰值下的理论最大幅度约为
  `+/-4.30 deg`；首次配置还会先触发 `2.0 N*m` 软限幅，因此算法实际可用幅度约为
  `+/-3.87 deg`，即约 `7.74 deg` 峰峰值。

`2.223 N*m` 是由驱动换算得到的名义短时命令峰值，不是连续热能力。仿真中的 5 Hz、
`+/-10 deg` 只验证饱和和恢复，不能用其 RMSE 否定或证明控制器设计。

## 9. 验收门禁

实现验收以逻辑正确和安全不变量为硬门禁，不以未校准对象模型上的性能提升为硬门禁：

1. 全部框架无关算法与路由测试通过；所有输出有限。
2. 对硬限幅已启用且 `Calculate()` 正常有效的周期，满足
   `torque_min <= tau_cmd <= torque_max`；软/硬/slew 顺序与标志正确。除进入重装和安全
   零输出外，slew 开启的正常控制周期变化率不超过配置值和浮点容差。RELAX、反馈失效
   和非法 `dt` 必须按设计立即撤除力矩，允许有意违反 slew 约束。
3. 非法 `dt`、非法目标、反馈失效、RELAX 和模式切换严格符合第 6、7 节。
4. 对全部有限输入，路由关闭时 AI Yaw 逐样本保持旧路径；路由开启但不处于有效 AI
   时，手动/巡逻 Yaw 和全部 Pitch 保持当前行为。NaN/Inf 被新增有限值保护拒绝属于
   明确的安全差异，不纳入旧路径等价性。
5. observer-only 配置与同一 LQR 配置的最终力矩逐样本相同，仅诊断状态变化。
6. 仿真报告必须完整展示当前 PID 和三个 LQR 增益候选，不隐藏饱和样本，也不把模型
   结果表述为实车提升。
7. 首次实验配置必须通过与“是否优于 PID”无关的有界性门禁：所有仿真满足
   `abs(e_theta) < pi`、`abs(omega) < 100 rad/s`；在名义对象、零扰动下，
   minimum-jerk 阶跃最后 1 秒的 `abs(e_theta) <= 1 deg` 且误差 RMS 不高于前 1 秒，
   名义正弦软/硬限幅样本并集占比小于 `50%`，过载回零后的最后 1 秒
   `abs(e_theta) <= 1 deg`、
   `abs(omega) <= 0.2 rad/s` 且不再触发软/硬限幅。其他候选失败时保留在报告中，但
   不得成为默认值。
8. `sentry_gimbal` 与 `sentry_chassis` 分板构建通过；所有包含 `Gimbal` 的配置
   `sentry`、`omni_infantry_3`、`omni_infantry_4`、`hero`、`aerial` 顺序编译通过。
9. 运行工程格式检查并保持 `-Werror` 构建。两份分板构建会重写同一生成文件，禁止
   并行执行。

性能指标只用于选择后续实车调参候选。没有辨识数据和实机测试时，本轮不能声称 RMSE、
相位或带宽获得确定比例的改善。

## 10. 实施与提交边界

预计生产修改范围：

- `Modules/Gimbal/Gimbal.hpp`
- `Modules/Gimbal/YawLqrEso.hpp`（新增）
- `Modules/Gimbal/CMakeLists.txt`（仅测试接入需要时）
- `Modules/Gimbal/tests/` 下的数学、仿真和静态回归测试
- `User/RobotConfig/sentry_gimbal.yaml`

`sentry_chassis.yaml` 只参与编译回归，不修改。`HostData`、`CMD`、`DualBoard`、
`RMMotor`、`DMMotor`、Pitch 配置和其他车型 YAML 不属于生产修改范围。

`Modules/Gimbal` 是独立且当前已有未提交改动的子仓库。实施必须在其当前内容上增量
修改，禁止回退、覆盖或把既有改动误算成本任务改动。模块提交与根仓库 YAML/文档提交
按现有仓库边界分别处理，生成文件不进入功能提交。

测试启用顺序仅用于降低风险，不形成生产阶段状态：

```text
基础：LQR + J*alpha + B*omega + observer-only ESO + 软/硬/slew
  -> 打开 ESO compensation 并检查门控
  -> 分别单独打开 Coulomb、LQI、torque bias
  -> 最后评估组合，不默认把全部开关同时置 true
```

最终交付是一个可编译、可离线验证、仅在目标 Sentry AI Yaw 生效的完整移植骨架，以及
明确标注为“推测起点”的参数。实车效果、最终增益和补偿组合仍需未来台架或实车数据
决定。
