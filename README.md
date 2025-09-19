# CMD 控制命令模块

## 模块简介
CMD(Control Command)模块负责处理来自不同控制源的命令，并将其转发到相应的执行单元。它是机器人控制系统的核心组件，连接输入设备(如遥控器、键盘鼠标)与执行单元(如底盘、云台)。

## 功能特性
- 支持多种控制源(遥控器、AI)的命令处理
- 提供控制模式切换(操作员控制、自动控制)
- 实现云台和底盘的分离控制
- 在线状态监测与丢失控制检测
- 事件映射与回调注册机制

## 数据结构
### 控制源
```cpp
typedef enum {
  CTRL_SOURCE_RC, /* 遥控器控制源 */
  CTRL_SOURCE_AI, /* AI控制源 */
  CTRL_SOURCE_NUM /* 控制源数量 */
} ControlSource;
```

### 控制模式
```cpp
typedef enum {
  CMD_OP_CTRL,    /* 操作员控制模式 */
  CMD_AUTO_CTRL,  /* 自动控制模式 */
} Mode;
```

### 底盘控制命令
```cpp
typedef struct {
  float x;        /* X轴方向控制量 */
  float y;        /* Y轴方向控制量 */
  float z;        /* Z轴方向控制量(旋转) */
} ChassisCMD;
```

### 云台控制命令
```cpp
typedef struct {
  LibXR::CycleValue<float> yaw; /* 偏航角(Yaw angle) */
  LibXR::CycleValue<float> pit; /* 俯仰角(Pitch angle) */
  LibXR::CycleValue<float> rol; /* 翻滚角(Roll angle) */
} GimbalCMD;
```

### 完整控制命令数据
```cpp
typedef struct {
  GimbalCMD gimbal;      /* 云台控制命令 */
  ChassisCMD chassis;    /* 底盘控制命令 */
  bool online;           /* 在线状态 */
  ControlSource ctrl_source; /* 控制源 */
} Data;
```

## 主要方法
- `RegisterController`: 注册控制器，接收来自指定源的数���
- `RegisterEvent`: 注册事件回调
- `SetCtrlSource`: 设置当前控制源
- `GetCtrlSource`: 获取当前控制源
- `GetCtrlMode`: 获取当前控制模式
- `Online`: 获取在线状态


## 数据流向
1. 控制设备(如DR16遥控器)将原始数据转换为CMD::Data格式
2. 通过RegisterController注册的回调，数据存入cmd->data_数组
3. 根据控制模式和控制源，将数据转发到相应的执行单元
4. 云台和底盘控制数据分别通过gimbal_data_tp_和chassis_data_tp_发布

## 依赖模块
无

## 硬件依赖
无（本模块仅处理逻辑，不直接操作硬件）
