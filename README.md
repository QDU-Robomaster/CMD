# CMD

控制命令中枢：汇总遥控器与上位机输入，发布底盘、云台、发射命令 / Control command hub that merges remote-controller and host inputs and publishes chassis, gimbal and launcher commands

## 1. 模块作用 / Purpose

CMD 把遥控器（DR16、VT13）与上位机等来源的输入整理为三路命令 Topic：底盘命令、云台命令、发射命令。下游 Module 订阅这三个 Topic，输入来源由 CMD 内部选择。

输入由其他 Module 调用 `FeedRC(...)` 或 `FeedAI(...)` 写入 `CMD::Data`，每次写入立即执行一轮处理并发布三路命令。处理、命令 Topic 的发布以及遥控器上线与离线事件的激活都在调用者的上下文中同步完成，Topic 与事件上注册的回调函数在同一上下文中运行。控制模式事件的回调 `EventHandler` 在激活该事件的上下文中运行，`SetCtrlMode(...)`、`GetCtrlMode()`、`Online()`、`GetAIGimbalStatus()` 在调用者的上下文中直接读写 CMD 的成员。

遥控器在线状态变化时，CMD 在自己的事件上激活 `CMD_EVENT_START_CTRL`（`0x13212508`，遥控器上线）或 `CMD_EVENT_LOST_CTRL`（`0x13212509`，遥控器离线）。

CMD merges the input of the remote controllers (DR16, VT13) and of the host into three command Topics: the chassis command, the gimbal command and the launcher command. Downstream Modules subscribe to these Topics, and CMD selects the input source internally.

Other Modules write a `CMD::Data` through `FeedRC(...)` or `FeedAI(...)`; every write runs one processing pass and publishes the three commands immediately. The processing, the publishing of the command Topics and the activation of the remote-controller online and offline events all complete synchronously in the context of the caller, and the callbacks registered on those Topics and events run in that same context. The callback `EventHandler` of the control mode events runs in the context that activates the event, and `SetCtrlMode(...)`, `GetCtrlMode()`, `Online()` and `GetAIGimbalStatus()` read and write the members of CMD directly in the context of the caller.

When the remote-controller online state changes, CMD activates `CMD_EVENT_START_CTRL` (`0x13212508`, remote controller online) or `CMD_EVENT_LOST_CTRL` (`0x13212509`, remote controller offline) on its own event.

## 2. 输入源与控制模式 / Input Sources and Control Modes

遥控输入分 DR16 与 VT13 两路：`FeedRC(RCInputSource, const Data&)` 指定输入源，`FeedRC(const Data&)` 使用 DR16。某一路写入的数据 `chassis_online` 为真，且存在操作时，该路成为活动源。存在操作指：底盘 `x`、`y`、`z` 或云台 `yaw`、`pit`、`rol` 的绝对值大于 0.05，底盘 `self_define` 不为 `NONE`，或 `launcher.isfire` 为真。活动源在线时持续使用；活动源离线后切换到另一路在线源；两路都离线时使用全零的离线数据。

控制模式决定发布的内容：

- `CMD_OP_CTRL`（操作手控制）：发布遥控数据。
- `CMD_AUTO_CTRL`（自动控制）：底盘命令在 AI 数据的 `chassis_online` 为真时取 AI 数据，否则取遥控数据；云台命令在 AI 数据的 `gimbal_online` 为真时取 AI 数据，否则取遥控数据；发射命令在 AI 与遥控的 `isfire` 同时为真时开火。

接口：

- `FeedRC(const Data&)`、`FeedRC(RCInputSource, const Data&)`：写入遥控数据。
- `FeedAI(const Data&)`：写入上位机或自动控制数据。
- `SetCtrlMode(Mode)`、`GetCtrlMode()`：设置、读取控制模式。
- `GetEvent()`：返回 CMD 的 `LibXR::Event`。在其上激活事件 ID `static_cast<uint32_t>(CMD::Mode::CMD_OP_CTRL)` 或 `static_cast<uint32_t>(CMD::Mode::CMD_AUTO_CTRL)` 即切换到对应的控制模式，`EventBinder` 等 Module 通过它绑定事件。
- `Online()`：遥控器是否在线。
- `GetAIGimbalStatus()`：AI 数据的 `gimbal_online`。

The remote-controller input has two channels, DR16 and VT13: `FeedRC(RCInputSource, const Data&)` names the input source and `FeedRC(const Data&)` uses DR16. A channel becomes the active source when the data written to it has `chassis_online` true and contains an operation. An operation means that the absolute value of the chassis `x`, `y`, `z` or of the gimbal `yaw`, `pit`, `rol` is greater than 0.05, that the chassis `self_define` is not `NONE`, or that `launcher.isfire` is true. The active source is used as long as it is online; when it goes offline, CMD switches to the other channel if that one is online; when both are offline, the all-zero offline data is used.

The control mode determines what is published:

- `CMD_OP_CTRL` (operator control): the remote-controller data is published.
- `CMD_AUTO_CTRL` (automatic control): the chassis command is taken from the AI data when its `chassis_online` is true, otherwise from the remote controller; the gimbal command is taken from the AI data when its `gimbal_online` is true, otherwise from the remote controller; the launcher fires when the `isfire` of both the AI data and the remote controller is true.

Interface:

- `FeedRC(const Data&)`, `FeedRC(RCInputSource, const Data&)`: write remote-controller data.
- `FeedAI(const Data&)`: write host or automatic control data.
- `SetCtrlMode(Mode)`, `GetCtrlMode()`: set and read the control mode.
- `GetEvent()`: returns the `LibXR::Event` of CMD. Activating the event ID `static_cast<uint32_t>(CMD::Mode::CMD_OP_CTRL)` or `static_cast<uint32_t>(CMD::Mode::CMD_AUTO_CTRL)` on it switches to the corresponding control mode; Modules such as `EventBinder` bind events through it.
- `Online()`: whether the remote controller is online.
- `GetAIGimbalStatus()`: the `gimbal_online` of the AI data.

## 3. 构造接口 / Constructor

```cpp
CMD(Mode mode = CMD::Mode::CMD_OP_CTRL,
    const char* chassis_cmd_topic_name = "chassis_cmd",
    const char* gimbal_cmd_topic_name = "gimbal_cmd",
    const char* launcher_cmd_topic_name = "launcher_cmd");
```

依赖：无。

配置参数：

- `mode`：初始控制模式，`CMD::Mode::CMD_OP_CTRL` 或 `CMD::Mode::CMD_AUTO_CTRL`，默认 `CMD_OP_CTRL`。
- `chassis_cmd_topic_name`：底盘命令 Topic 名称，默认 `"chassis_cmd"`。
- `gimbal_cmd_topic_name`：云台命令 Topic 名称，默认 `"gimbal_cmd"`。
- `launcher_cmd_topic_name`：发射命令 Topic 名称，默认 `"launcher_cmd"`。

Dependencies: none.

Configuration parameters:

- `mode`: initial control mode, `CMD::Mode::CMD_OP_CTRL` or `CMD::Mode::CMD_AUTO_CTRL`, default `CMD_OP_CTRL`.
- `chassis_cmd_topic_name`: name of the chassis command Topic, default `"chassis_cmd"`.
- `gimbal_cmd_topic_name`: name of the gimbal command Topic, default `"gimbal_cmd"`.
- `launcher_cmd_topic_name`: name of the launcher command Topic, default `"launcher_cmd"`.

## 4. Topic

三个 Topic 均以多发布者模式创建。

| Topic（默认名） | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `chassis_cmd` | 发布 | `CMD::ChassisCMD` | 底盘命令：`x`、`y`、`z`（旋转）控制量与 `self_define`（`NONE`、`BOOST`、`STRETCH`） |
| `gimbal_cmd` | 发布 | `CMD::GimbalCMD` | 云台命令：`yaw`、`pit`、`rol` 及其一阶、二阶导数 |
| `launcher_cmd` | 发布 | `CMD::LauncherCMD` | 发射命令：`isfire` |

All three Topics are created in multi-publisher mode.

| Topic (default name) | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `chassis_cmd` | Publish | `CMD::ChassisCMD` | Chassis command: `x`, `y`, `z` (rotation) control values and `self_define` (`NONE`, `BOOST`, `STRETCH`) |
| `gimbal_cmd` | Publish | `CMD::GimbalCMD` | Gimbal command: `yaw`, `pit`, `rol` and their first and second derivatives |
| `launcher_cmd` | Publish | `CMD::LauncherCMD` | Launcher command: `isfire` |

## 5. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/CMD` 写入的实例，各项为默认值：

An instance written by `xrobot instance add QDU-Robomaster/CMD`, with all values at their defaults:

```yaml
modules:
  - module: QDU-Robomaster/CMD
    id: cmd
    args:
      - mode: CMD::Mode::CMD_OP_CTRL
      - chassis_cmd_topic_name: "chassis_cmd"
      - gimbal_cmd_topic_name: "gimbal_cmd"
      - launcher_cmd_topic_name: "launcher_cmd"
```

DR16、VT13、Gimbal 等 Module 以 `CMD&` 参数引用 CMD，配置中填写 CMD 的实例 id（上例为 `cmd`），CMD 实例列在它们之前。

Modules such as DR16, VT13 and Gimbal take CMD as a `CMD&` parameter; the configuration holds the instance id of CMD (`cmd` above), and the CMD instance is listed before them.

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：无。输入数据由 DR16、VT13 等 Module 写入。

Dependencies: LibXR.

Hardware: none. The input data is written by Modules such as DR16 and VT13.
