# HostData

## 1. 模块作用
上位机数据接入模块。把主机侧目标转换为 CMD 输入。

## 2. 主要函数说明
1. HostCMD: 汇总云台/底盘/发射数据并喂给 CMD。
2. 构造函数中的三个 Topic 回调: 接收 euler、chassis、fire 数据。
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
  - host_euler_topic_name: "target_eulr"
  - host_chassis_data_topic_name: "host_chassis_data"
  - host_fire_topic_name: "host_fire_notify"
template_args:
[]

## 5. 依赖与硬件
Required Hardware:
[]

Depends:
[]

## 6. 代码入口
Modules/HostData/HostData.hpp
