# HostData

## 1. 模块作用
上位机数据接入模块。把主机侧目标转换为 CMD 输入。

## 2. 主要函数说明
1. ThreadFunc: 每 5 ms 接收云台、底盘、发射 Topic 数据，检查超时并更新 CMD。
2. BuildHostCMD: 汇总有效数据；底盘输入校验、超时及重新使能逻辑均位于 HostData.hpp。
3. OnMonitor: 监控钩子（当前为空实现）。

## 3. 接入步骤
1. 添加模块并配置 host 侧 Topic 名称。
2. 保证上位机发布数据结构与本模块一致。
3. 检查 CMD 是否收到 AI 控制输入。

标准命令流程：
    xrobot_add_mod HostData --instance-id hostdata
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 配置示例（YAML）
module: HostData
entry_header: Modules/HostData/HostData.hpp
constructor_args:
  - cmd: '@cmd'
  - host_gimbal_topic_name: "target_euler"
  - host_chassis_data_topic_name: "host_chassis_data"
  - host_fire_topic_name: "host_fire_notify"
  - task_stack_depth: 1024
  - thread_priority: LibXR::Thread::Priority::MEDIUM
template_args:
[]

## 5. 依赖与硬件
Required Hardware:
[]

Depends:
- pldx/CMD
- pldx/HostDataLink

## 6. 代码入口
Modules/HostData/HostData.hpp

## LibXR 2.0 静态装配

使用 xrobot 1.0.1 的 `module` / 有序 `args`；构造函数显式注入外设与服务，实例按依赖顺序创建。参数名和顺序以头文件中的构造函数为准，固件示例见 `User/RobotConfig/`。

保留 PLDX 控制、协议、超时与离线保护。 `OnMonitor()` 由生成的监控循环直接调用。
