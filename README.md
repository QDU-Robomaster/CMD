# CMD

用于统一汇总控制输入并发布控制命令的中枢模块。

CMD 把不同来源（遥控器 DR16 / VT13、上位机等）的输入统一整理为三路命令 Topic：
底盘命令、云台命令、发射命令。下游模块只订阅 Topic，不关心输入来源。

## 行为

- 输入模块调用 `FeedRC(...)` 或 `FeedAI(...)` 喂入 `CMD::Data`；每次喂入都会立即执行
  `ProcessAndPublish()` 并发布三路命令。CMD 自身不创建线程，没有输入时不发布。
- 遥控输入分 DR16 与 VT13 两路（`FeedRC(RCInputSource, const Data&)`；
  `FeedRC(const Data&)` 按 DR16 处理）。某一路在线（`chassis_online`）且有操作
  （摇杆绝对值 > 0.05、自定义底盘模式或开火）时成为活动源；活动源离线后切换到另一路在线源；
  两路都离线时使用离线（全零）数据。
- 控制模式：
  - `CMD_OP_CTRL`（操作手）：直接发布遥控数据。
  - `CMD_AUTO_CTRL`（自动）：底盘 / 云台命令在 AI 数据的 `chassis_online` /
    `gimbal_online` 为真时取 AI 数据，否则取遥控数据；开火需要 AI 和遥控同时开火。
- 遥控在线状态变化时在 CMD 的事件上触发 `CMD_EVENT_START_CTRL`（`0x13212508`）或
  `CMD_EVENT_LOST_CTRL`（`0x13212509`）。

## Topic

| Topic（默认名） | 类型 | 内容 |
| --- | --- | --- |
| `chassis_cmd` | `CMD::ChassisCMD` | `x`、`y`、`z`（旋转）控制量与 `self_define`（`NONE` / `BOOST` / `STRETCH`） |
| `gimbal_cmd` | `CMD::GimbalCMD` | `yaw`、`pit`、`rol` 及其一阶、二阶导数 |
| `launcher_cmd` | `CMD::LauncherCMD` | `isfire` |

三个 Topic 都以多发布者模式创建。

## 接口

- `FeedRC(const Data&)` / `FeedRC(RCInputSource, const Data&)`：喂入遥控数据。
- `FeedAI(const Data&)`：喂入上位机 / 自动控制数据。
- `SetCtrlMode(Mode)` / `GetCtrlMode()`：切换 / 读取控制模式。
- `GetEvent()`：返回 CMD 的 `LibXR::Event`。在其上激活事件 ID
  `static_cast<uint32_t>(CMD::Mode::CMD_OP_CTRL)` 或 `CMD_AUTO_CTRL` 会切换模式
  （`EventHandler()`）；EventBinder 等模块通过它绑定事件。
- `Online()`：遥控是否在线；`GetAIGimbalStatus()`：AI 数据的 `gimbal_online`。

## 依赖

无其他模块依赖，仅使用 LibXR。DR16、VT13 以及底盘、云台、发射等模块依赖本模块。

## 构造接口

```cpp
CMD(Mode mode = CMD::Mode::CMD_OP_CTRL,
    const char* chassis_cmd_topic_name = "chassis_cmd",
    const char* gimbal_cmd_topic_name = "gimbal_cmd",
    const char* launcher_cmd_topic_name = "launcher_cmd");
```

无依赖参数。

配置：

- `mode`：初始控制模式，`CMD::Mode::CMD_OP_CTRL` 或 `CMD::Mode::CMD_AUTO_CTRL`，
  默认 `CMD_OP_CTRL`。
- `chassis_cmd_topic_name`：底盘命令 Topic 名，默认 `"chassis_cmd"`。
- `gimbal_cmd_topic_name`：云台命令 Topic 名，默认 `"gimbal_cmd"`。
- `launcher_cmd_topic_name`：发射命令 Topic 名，默认 `"launcher_cmd"`。

## 使用

```sh
xrobot module add QDU-Robomaster/CMD
xrobot setup
xrobot instance add QDU-Robomaster/CMD
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，默认值按源码写出（CMD 没有
依赖参数）：

```yaml
modules:
  - module: QDU-Robomaster/CMD
    id: cmd_0
    args:
      - mode: CMD::Mode::CMD_OP_CTRL
      - chassis_cmd_topic_name: '"chassis_cmd"'
      - gimbal_cmd_topic_name: '"gimbal_cmd"'
      - launcher_cmd_topic_name: '"launcher_cmd"'
```

CMD 不需要 BSP 对象，因此不需要 `XR_REGISTER`。其他模块（如 DR16、VT13）以
`CMD&` 参数引用它时填写 CMD 的实例 id（上例为 `cmd_0`），并且 CMD 必须在 `modules:`
中排在它们前面。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/CMD`
（在 BSP 中）打印 manifest 和当前的构造函数。
