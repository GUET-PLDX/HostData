# 云台控制链性能优化设计

## 1. 目标与范围

本设计针对现有 XRobot/LibXR 工程中的 RoboMaster 两轴云台控制链，首验对象为
`sentry_gimbal.yaml`，硬件为 GM6020 Yaw、DM4310 Pitch、云台 BMI088，以及底盘
控制板 BMI088 提供的实测 `gyro_z`。首验通过后再迁移到其他车型。

目标是改善动态跟踪、控制时序和小陀螺状态下的云台稳定性，同时保持现有模块边界、
Topic 语义和 YAML 驱动方式。

本设计明确不做以下事情：

- 不新增 `GimbalController.hpp`、`Telemetry`、`ChassisMotion` 或公共控制 DTO。
- 不修改 Pitch 重力补偿公式、`pit_lc`、`pit_theta` 的含义。
- 不在首验阶段把控制频率提升到 1 kHz。
- 不在首验阶段全局删除 `Gimbal.hpp` 中的 legacy 轴取反。
- 不增加高阶 Coulomb/二次阻力模型；Yaw 先沿用线性 `yaw_k`。
- 不实现底盘角加速度补偿、`chassis_alpha_z` 或 `rotor_accel_k`。
- 不增加 Gimbal 独立扭矩限幅参数或逐项前馈限幅。
- 不为 Ozone 增加控制逻辑不需要的镜像成员或统计结构。
- 不手工修改生成的 `User/xrobot_main.hpp`。

Ozone 负责采集现有类成员并导出完整时间序列，实时控制线程不计算 RMS、p95 或
窗口统计。

### 1.1 实现约束

实现采用 LibXR-first 原则，并保持各文件原有代码书写风格：

- 调度、时间戳、Topic、回调、互斥、队列、错误码、PID 和周期量优先使用现有
  `LibXR::Thread`、`LibXR::Timebase`、`LibXR::Topic`、`LibXR::Callback`、
  `LibXR::Mutex`、`LibXR::MPMCQueue`、`LibXR::ErrorCode`、`LibXR::PID` 和
  `LibXR::CycleValue`，不重复实现同类基础设施。
- 只有 LibXR 没有合适接口时才使用 C++ 标准库或文件内局部实现，并在实施计划中
  说明原因；不得因此修改 `Middlewares/Third_Party/LibXR`。
- 保持现有类、方法、成员、私有工具函数和控制流程的组织方式；新逻辑直接增量落在
  `Gimbal.hpp`、`DualBoard.hpp` 及既有电机文件中，不引入新的控制层或公共抽象。
- 保持模块 manifest 与 YAML 构造参数一一对应，不手工修改生成代码；新增参数必须有
  向后兼容默认值。
- 遵守工程现有命名和格式：成员变量使用尾下划线，常量使用大写命名，方法沿用
  CamelCase，并使用工程指定的 clang-format 版本格式化 `Modules/` 范围。
- 不为了统一写法重构任务范围外的代码；每项改动应尽量贴近所在文件的既有实现。

## 2. 固定控制管线

控制顺序保持不变：

```text
命令/传感器快照
  -> Gimbal::Update()
  -> Gimbal::ParseCMD()
  -> Gimbal::Control()
  -> Gimbal::Solve()
  -> Motor::Control()
```

观测旁路只有：

```text
Gimbal 成员变量 -> Ozone -> 时间序列 -> 离线分析
```

Gimbal 不依赖 `DualBoard` 或任何 `Chassis` 类型。底盘运动只通过基础 Topic
输入，避免控制模块与车型模板耦合。

## 3. 坐标与兼容策略

工程目标坐标系为：`+X` 车体前、`+Y` 车体左、`+Z` 向上，Yaw 左转为正，Pitch
上抬为负。根据云台 C 板 USB 口朝车体前方推导出的候选变换为：

```text
q_BS = (0.70710678, 0, 0, -0.70710678)
R_BS = [[ 0, 1, 0],
        [-1, 0, 0],
        [ 0, 0, 1]]
```

该变换只作为 T0 实测候选值，不在测得 BMI088 芯片轴向前写入 YAML。首验采用保守
兼容策略：保留 `Gimbal.hpp` 当前 Euler Pitch 和 gyro-Y 的 legacy 手工取反，仅在
T0 记录并验证标准坐标的符号。不得只修改 sentry YAML
后删除共享代码中的取反，因为 `gimbal_euler` 还被底盘 FOLLOW、DualBoard 和
其他模块消费，`gimbal_gyro` 也可能被其他车型使用。

底盘 BMI088 的安装方向必须单独实测，不能复制云台的四元数。底盘坐标门禁未通过
前，底盘前馈始终关闭。完整坐标迁移作为后续逐车型任务，需同时回归 AHRS、Gimbal、
DualBoard attitude、底盘 FOLLOW 和 Pitch 重力行为。

## 4. 控制律设计

### 4.1 人工目标速度前馈

保持现有死区、方向和灵敏度，只改变目标生成语义。

人工控制时：

```text
operator_rate = command * GIMBAL_MAX_SPEED * sensitivity
target_angle += operator_rate * dt
target_rate_ff = operator_rate
target_ddot = 0
```

低灵敏度、普通手动和非 AI 自动分支分别保留当前符号约定；自动巡航分支必须显式
写入目标速度和加速度，不能残留上一控制模式的值。AI 分支继续使用命令中携带的
角度、角速度和角加速度。

### 4.2 加速度前馈去重

当前实现对包含 `target_*_dot` 的完整目标速度求差分，又叠加 `target_*_ddot`，
会在摇杆阶跃时重复计算加速度。改为保存角度环输出的独立历史量：

```text
angle_loop_omega = PID_angle(angle_error)
target_omega     = angle_loop_omega + target_rate_ff
alpha_I_cmd      = derivative(angle_loop_omega) + target_ddot
inertia_ff       = J * alpha_I_cmd
```

复用现有 `target_yaw_dot_`、`target_pit_dot_` 保存目标速度前馈，不增加另一组
`target_*_rate_ff_` 成员。`last_*_angle_loop_omega_` 在异常 `dt`、模式切换和 IMU
无效时重置。Pitch 重力
项保持原式：

```text
gravity_ff_pit = -pit_lc * sin(Pitch + pit_theta)
```

### 4.3 小陀螺速度前馈

云台 BMI088 已提供惯性角速度反馈，因此底盘角速度不能再次直接加到惯性速率误差。
底盘项只进入执行器侧相对速度模型：

```text
rotor_ff_active = rotor_ff_enabled &&
                  dualboard_chassis_mode == ROTOR
omega_motor_ref = rotor_ff_active
                    ? omega_I_cmd - chassis_gyro_z
                    : omega_I_cmd

yaw_feedforward = j_yaw * alpha_I_cmd
                + yaw_k * omega_motor_ref

tau_yaw = PID_rate(omega_I_cmd - gimbal_gyro_z,
                   feedforward = yaw_feedforward)
```

进入 ROTOR 后直接以完整底盘角速度补偿，整个 ROTOR 期间保持全量启用；退出 ROTOR
后直接关闭。不增加 `rotor_weight`、数据年龄分段、渐入或渐出状态机。PPT 6.5
P12--P14 面向相对电机位置闭环，要求把底盘补偿同时写入目标角增量和目标速度；当前
工程使用惯性 Euler 和惯性 gyro 闭环，目标惯性角本身已保持不变，因此不再创建
`yaw_joint_ref`。等效补偿只改变执行器相对速度 `omega_motor_ref`，避免把底盘 gyro
再次加入惯性速率误差而双重补偿。

### 4.4 LibXR PID 前馈与限幅

惯量、Yaw 阻力和 Pitch 重力项通过 `LibXR::PID::SetFeedForward()` 写入既有速度环，
由现有 `pid_yaw_omega.out_limit` 和 `pid_pit_omega.out_limit` 对反馈与前馈的总和限幅。
首验 sentry 分别使用 GM6020 和 DM4310 的物理边界作为现有 PID `out_limit` 初值，
最终仍由 RMMotor/DMMotor 驱动层 clamp 兜底。

Gimbal 不增加 `yaw_torque_limit`、`pit_torque_limit` 或
`yaw_ff_torque_limit`。每个控制周期的顺序固定为“计算前馈 ->
`SetFeedForward()` -> `Calculate()`”，避免前馈滞后一周期；进入 RELAX 或重置 PID 前
先将 feedforward 置零，避免 `PID::Reset()` 后残留旧前馈。Pitch 重力公式本身保持
不变，只改变其进入速度环的方式。

## 5. 底盘运动数据链

### 5.1 Topic 边界

底盘侧新增 BMI088 实例并沿用 BMI088 模块现有的三轴 gyro Topic；`DualBoard` 只订阅
该本地数据并提取 `gyro_z`。跨板后只新增一个基础 Topic：

```text
chassis_gyro_z: float, rad/s
```

Gimbal 在控制线程启动前分别取得两个 Topic 句柄，异步订阅者直接使用句柄，避免按
名称永久等待：

- 新增 `chassis_gyro_z`：Gimbal 与 DualBoard 两端都使用
  `FindOrCreate<float>(..., nullptr, false)`；Gimbal 自身不发布，DualBoard 发布端不得
  复用现有强制 `multi_publisher=true` 的 helper。
- 既有 `dualboard_chassis_mode`：Gimbal 使用
  `FindOrCreate<uint32_t>(..., nullptr, true)` 预创建，保持 DualBoard 现有多发布者语义。

`chassis_gyro_z_` 初始为 0，`dualboard_chassis_mode_` 初始为 RELAX。这样既兼容
`sentry_gimbal.yaml` 中 Gimbal 先于 DualBoard 的构造顺序，也不会让无 DualBoard 的
车型卡在 `WaitTopic` 或因 Topic mutex 语义不一致触发 ASSERT。

ROTOR 判断直接复用现有 `dualboard_chassis_mode`，不增加
`chassis_rotor_active` Topic。这里使用的是云台板已发出的本地模式请求，不把它描述成
底盘执行状态回读；真实模式回读若有需要，作为后续协议升级处理。Gimbal 只依赖两个
基础 Topic 的数值，不依赖 `DualBoard` 或 `Chassis` 类型。

### 5.2 CAN MotionFrame

沿用 `DualBoard.hpp` 已有私有 8 字节帧风格，不新增模块或 CMake 文件：

```text
CAN ID: CHASSIS 角色 tx_id + 0x10（sentry 底盘为 0x321）
周期: 10 ms

int16 gyro_z_q
uint8 gyro_valid
uint8 reserved[5]
```

陀螺单位为 rad/s，固定量化比例为 `GYRO_SCALE = 900.0f` LSB/(rad/s)，可表示
±36.4 rad/s，覆盖 BMI088 的 ±2000 deg/s 量程。编码显式检查有限值和范围，合法时
量化 `gyro_z_q` 并置 `gyro_valid=1`，非法时发送零值并置
`gyro_valid=0`；`reserved` 全部清零，不使用 C bitfield。读取 BMI088 快照和更新帧缓存
沿用 `data_mutex_`，不引入新的并发抽象。

10 ms 周期与 `DualBoard` 现有 `CONTROL_PERIOD_MS` 一致，不额外增加 5 ms 发送任务。
MotionFrame 的接收复用既有 `last_rx_time_ms_`、`online_` 和
`safe_state_published_`，将底盘板的 10 ms 帧作为现有双板链路心跳；不增加
MotionFrame 专用时间戳或第二套离线状态机。云台端收到显式无效帧时发布零 gyro，
MotionFrame 停止并超过既有 `offline_timeout_ms` 时也沿现有离线路径发布零 gyro。

### 5.3 直接使用与已接受风险

Gimbal 收到有效 MotionFrame 后直接保存最新 `chassis_gyro_z_`。ROTOR 期间始终使用
该值，不计算 motion age，不增加独立的软/硬离线门限，也不做权重变化。DualBoard
既有 `offline_timeout_ms` 和离线处理保持原样；完整双板链路离线时，沿既有离线路径
发布零 gyro。

该简化方案明确接受一个残余风险：如果只有 MotionFrame 停更，而其他 DualBoard
反馈帧仍持续刷新同一条链路时间戳，Gimbal 将继续使用最后一次
`chassis_gyro_z_`。首验不为这一局部停更场景增加第二套超时状态机。

MotionFrame 不携带序号、模式或源时间戳。帧布局、CAN ID、发送周期和数值符号通过
现有静态回归、CAN analyzer 和 Ozone 台架检查；不新增 CAN replay 框架。

## 6. 时序与失效安全

当前 `Gimbal::ThreadFunc()` 使用 `Sleep(2)`。在 1 kHz FreeRTOS tick 下，实际循环
周期是“本轮计算和抢占时间 + 2 tick”，不是固定 2 ms，也不能直接视为严格 500 Hz。
首验保留 `Sleep(2)`，只用 Ozone 记录至少 30 s 的实际 `dt` 分布，不增加
`SleepUntil` A/B，也不把 `dt == 2 ms` 当成验收事实。

数值保护使用宽于正常调度的窗口：`0.5 ms < dt <= 20 ms`。超出窗口时本周期不做
目标积分、PID 微分或惯量前馈，目标角保持不变，速度 PID feedforward 显式置零，
本周期输出零扭矩并重置导数历史；下一次合法周期直接恢复，不增加额外状态机。不能把
异常 `dt` 简单 clamp 后继续积分。

云台 IMU 只使用一个硬门限：Euler 或 gyro 从未收到、包含非有限值，或任一数据超过
50 ms 未更新时，Gimbal 进入 RELAX。没有 warning/stale/硬离线三段状态，也不按 IMU
age 单独降级某一项前馈。实现只增加保护逻辑真正需要的
`euler_received_`、`gyro_received_`、`last_euler_update_`、`last_gyro_update_` 等价成员，
不封装 Telemetry 结构，也不要求 Ozone 默认采集这些时间戳。

电机 freshness 独立于 rotor 算法：

- `DMMotor::Update()` 保留 200 ms 首次反馈宽限；收到首帧后，150 ms 未更新返回
  `LibXR::ErrorCode::NO_RESPONSE`。
- `RMMotor::Update()` 将循环次数判断替换为同一 150 ms 时间硬门限。
- Gimbal 不重复维护电机时间戳，只消费两个 `Motor::Update()` 返回值；任一个非 `OK`
  就进入 RELAX，不使用旧反馈。
- 显式无效 MotionFrame 或既有 DualBoard 完整离线路径只把底盘 gyro 置零，不影响
  云台自身 IMU 闭环。

## 7. 分阶段实施门禁

### T0：基线与坐标候选验证（无代码）

冻结供电、功率限制、温度和测试速度，采集静止、人工移动和 ROTOR 三组 Ozone
基线。记录 `Sleep(2)` 下的实际 `dt` 分布；实测云台 BMI088 的数据符号，并确认底盘
BMI088 的物理安装轴向，验证第 3 节 `q_BS` 只能作为候选。首验继续保留 legacy
Pitch/gyro-Y 取反且云台行为不得偏离基线；底盘 BMI 的运行时单位/符号在 T3 配置实例
后验证，完整七角度、FOLLOW 和 attitude 坐标回归留到真正的坐标迁移任务。

### T1：Gimbal 核心控制

仅增量修改 `Gimbal.hpp`：人工命令同时积分目标角并写入现有 `target_*_dot_`，分离
角度环输出的微分以消除加速度重复计算，通过 `LibXR::PID::SetFeedForward()` 注入
惯量/Yaw 阻力/Pitch 重力前馈，并加入 `dt` 与单一 IMU 硬保护。固定底盘完成旧版/新版
A/B，确认 Pitch 重力回归通过后再进入下一阶段。

### T2：电机驱动 freshness

分别在 `DMMotor.hpp` 和 `RMMotor.hpp` 实现驱动层时间硬门限，再由 Gimbal 继续只消费
`Motor::Update()`。拔除 DM4310、GM6020 反馈，验证 150 ms 门限和 RELAX；DM 上电无
反馈场景另验证 200 ms 启动宽限。

### T3：底盘 BMI088 与 MotionFrame 影子链路

在 `sentry_chassis.yaml` 增加底盘 BMI088，向 `DualBoard.hpp` 增加最小 MotionFrame，
并让 Gimbal 接收 `chassis_gyro_z`，但保持 `rotor_ff_enabled=false`。现有静态回归只
检查 8 字节布局、0x321 ID、量化和 Topic；CAN analyzer/Ozone 验证正反转符号、显式
无效帧和完整双板离线归零，并确认车体左转时底盘 `gyro_z > 0`。

### T4：ROTOR 速度前馈 A/B

在固定底盘速度下先辨识 signed `yaw_k`，再只对 sentry 开启
`rotor_ff_enabled`。正向和反向小陀螺必须分别通过 gyro、相对速度和扭矩符号检查。
ROTOR 内全程直接使用完整 `chassis_gyro_z_`，退出 ROTOR 立即关闭；不测试渐退、age
权重或角加速度项。

### T5：云台车型迁移与全量编译

只迁移实际使用 `Gimbal` 的配置，顺序为 `sentry_gimbal` -> `sentry` ->
`omni_infantry_3` -> `omni_infantry_4` -> `hero` -> `aerial`。逐车型审计 Topic、BMI
变换、电机方向、Pitch 限位、Yaw 阻力系数和速度 PID `out_limit`；没有可用底盘
`gyro_z` 的车型保持 `rotor_ff_enabled=false`。最后运行九个配置的编译回归：

```text
aerial, dart, helm_infantry, hero, omni_infantry_3,
omni_infantry_4, radar, sentry, wheel_leg
```

## 8. Ozone 指标与验收门限

Ozone 只采集实际参与控制的成员：`dt_`、`target_*_cmd_`、`target_*_dot_`、
`target_*_ddot_`、`euler_`、`gyro_data_`、`motor_*_feedback_`、
`motor_feedback_online_`、`chassis_gyro_z_`、保存现有模式 Topic 数值的成员，以及最终
被用于 `Motor::MotorCmd` 的 `yaw_output_`、`pit_output_`。最终输出必须直接作为控制值
使用，不能只是 Ozone 镜像。

不增加 `Telemetry`、`rotor_ff_active_`、逐项前馈扭矩、执行时间、丢帧计数或饱和
状态镜像；离线分析可根据已知公式、PID `out_limit` 和最终输出重建相关指标。

所有 A/B 使用相同供电、温度、功率限制、动作轨迹和重复次数，交替执行 A/B，离线
计算角度误差 RMS/p95、超调、速度误差、最终输出 RMS/峰值、相位延迟和饱和比例。
人工阶跃和 ROTOR 正反转每组至少重复 5 次。T1 先提交加速度去重、PID 前馈入口、
`out_limit` 和数值保护，再单独提交人工 `target_*_dot_`；人工速度前馈 A/B 分别构建这
两个相邻 commit，使唯一变量是人工目标速度前馈。原始基线与完整 T1 候选另做行为回归，
不把多项变化归因给单一机制。T4 使用同一候选代码，分别以
`rotor_ff_enabled=false/true` 构建，不再增加第二个运行时开关。

首验初始门限如下；正式调参前可依据冻结基线修订，但不能在看见结果后临时放宽：

| 指标 | 门限 |
|---|---|
| 控制周期 | 首验只报告 p50/p95/p99 和 `dt > 4 ms` 次数；`dt <= 0.5 ms` 或 `dt > 20 ms` 触发数值保护 |
| 云台 IMU | Euler 或 gyro 超过 50 ms 未更新，或出现非有限值，进入 RELAX |
| 电机反馈 | 首帧后 150 ms 未更新返回 `NO_RESPONSE`；DM 首帧前宽限 200 ms |
| 底盘 MotionFrame | 标称 10 ms；有效帧数值误差不超过 1 LSB，不设置独立 age/渐退/硬离线门限 |
| 人工速度前馈 | 动态角误差 RMS 下降至少 15%，超调增加不超过 10% |
| ROTOR 速度前馈 | Yaw 误差 RMS 下降至少 20%，非 ROTOR 指标恶化不超过 5% |
| Pitch 重力 | 静态误差和保持扭矩相对基线变化不超过 5% |
| 饱和 | 正常动作饱和比例 < 1%，峰值不超过物理限幅 |

Pitch 重力公式不允许因门限未达标而改写；只能回退其他新增前馈。

## 9. 文件与提交边界

生产代码的修改范围限制为现有文件：

- Gimbal nested repo：`Modules/Gimbal/Gimbal.hpp`。
- 电机 nested repo：`Modules/DMMotor/DMMotor.hpp`、
  `Modules/RMMotor/RMMotor.hpp`。
- DualBoard 源文件：`Modules/DualBoard/DualBoard.hpp` 和现有
  `tests/dualboard_static_regression.ps1`。
- 首验配置：`User/RobotConfig/sentry_gimbal.yaml`、
  `User/RobotConfig/sentry_chassis.yaml`。
- 迁移配置：`sentry.yaml`、`omni_infantry_3.yaml`、`omni_infantry_4.yaml`、
  `hero.yaml`、`aerial.yaml`。

不新增生产代码文件。Gimbal manifest 只新增一个构造参数
`rotor_ff_enabled`，默认 `false`；不为 Topic 名、timeout、扭矩限幅或 A/B 增加其他
构造参数。

`Gimbal`、`DMMotor`、`RMMotor` 在现有独立模块仓库中分别提交。当前
`Modules/DualBoard/` 被根仓库忽略且不是独立仓库，因此 T3 的前置门禁是取得或建立
DualBoard 的权威模块源仓库 checkout，在该仓库提交 MotionFrame，并让当前模块源索引
能够按该 commit/tag 重新获取相同版本；不能只修改本地生成副本。若现有模块源机制不能
固定该版本，先以独立工具链提交补齐可复现获取方式，再继续 T3。

本轮不改变模块清单，因此不修改 `Modules/modules.yaml`。根工程只提交实际 YAML、
静态回归和可复现的模块版本落点，生成文件与功能提交分开；不得回滚或混入工作区已有
的无关修改。

## 10. 运行时关闭与提交级回退

本轮只保留一个运行时开关，其余按独立 commit 回退：

- `rotor_ff_enabled=false`：完全关闭底盘速度前馈。
- 回退 T1 的独立 Gimbal commit：恢复人工分支原有的 `target_*_dot_=0`，同时保留目标
  角积分；不增加第二个运行时开关。
- `pid_yaw_omega.out_limit=0`、`pid_pit_omega.out_limit=0`：恢复 LibXR PID 原有的
  “0 表示不限幅”行为；电机驱动物理 clamp 始终保留。

只有 T0--T4 全部通过，才进入 T5 车型迁移。

## 11. 明确延期项

以下内容不属于本轮首验；只有 A/B 数据证明存在对应问题时再立独立任务：

- 底盘角加速度补偿、gyro 差分滤波、死区和额外限幅；仅当 T4 数据显示残差与底盘
  角加速度显著相关时再评估。
- `SleepUntil`、严格 1 kHz 控制周期或控制执行时间预算。
- 全车型坐标标准化，以及 legacy Pitch/gyro-Y 取反清理。
- 真实底盘执行模式回读。
- MotionFrame 独立 timeout、age 权重、渐入渐出和局部掉帧恢复。
- Coulomb、二次阻力或其他高阶摩擦模型。
