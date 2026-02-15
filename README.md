# CMD

用于统一汇总控制输入并发布控制命令的中枢模块。

CMD 把不同来源（例如遥控、上位机）统一整理为三路命令：

1. 底盘命令
2. 云台命令
3. 发射命令

## 解决什么问题

CMD 主要解决三件事：

1. 统一控制源输入接口（`FeedRC` / `FeedAI`）。
2. 统一控制模式切换（操作手/自动等）。
3. 统一命令发布出口（下游模块只订阅 Topic，不关心输入来源）。

## 核心流程

CMD 的典型流程是：

1. 输入模块调用 `FeedRC(...)` 或 `FeedAI(...)`。
2. CMD 内部执行 `ProcessAndPublish()`。
3. 发布 `chassis_cmd`、`gimbal_cmd`、`launcher_cmd`。
4. 底盘/云台/发射模块各自订阅并执行。

关键函数：

1. `FeedRC(const Data&)`：喂入遥控控制数据。
2. `FeedAI(const Data&)`：喂入上位机/自动控制数据。
3. `SetCtrlMode(Mode)`：切换控制模式。
4. `EventHandler(uint32_t)`：响应外部事件切换模式。
5. `ProcessAndPublish()`：统一整理并发布命令。
6. `GetEvent()`：提供事件绑定入口给 EventBinder 等模块使用。

## 最小接入示例

1. 添加模块：

```bash
xrobot_add_mod CMD --instance-id cmd
xrobot_gen_main
```

2. 典型配置（来自模块 manifest）：

```yaml
module: CMD
entry_header: Modules/CMD/CMD.hpp
constructor_args:
  - mode: CMD::Mode::CMD_OP_CTRL
  - chassis_cmd_topic_name: "chassis_cmd"
  - gimbal_cmd_topic_name: "gimbal_cmd"
  - launcher_cmd_topic_name: "launcher_cmd"
template_args: []
```

3. 下游模块只需订阅对应命令 Topic。

## 使用约定

1. 推荐先初始化 CMD，再初始化依赖它的控制模块。
2. 控制源切换尽量都走 CMD 的 `SetCtrlMode` / 事件入口。
3. 不同来源数据结构最终都转换成 `CMD::Data` 再进入 CMD。

## 模块信息

1. 代码入口：`Modules/CMD/CMD.hpp`
2. Required Hardware：None
3. Constructor Arguments：
   - `mode`
   - `chassis_cmd_topic_name`
   - `gimbal_cmd_topic_name`
   - `launcher_cmd_topic_name`
4. Template Arguments：None
5. Depends：None
