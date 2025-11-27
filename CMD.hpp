#pragma once

/* clang-format off */
/* === MODULE MANIFEST V2 ===
module_description: 控制命令模块
constructor_args:
  - mode: CMD::Mode::CMD_OP_CTRL
  - chassis_cmd_topic_name: "chassis_cmd"
  - gimbal_cmd_topic_name: "gimbal_cmd"
  - launcher_cmd_topic_name: "launcher_cmd"
=== END MANIFEST === */
/* clang-format on */

/**
 * @file CMD.hpp
 * @brief 控制命令处理模块
 * @details 负责处理来自不同控制源的命令，并将其转发到相应的执行单元
 */

#include <vector>

#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "message.hpp"

/**
 * @class CMD
 * @brief 控制命令处理类
 * @details 接收来自不同控制源的命令，处理并转发到底盘和云台等执行单元
 */
class CMD : public LibXR::Application {
 public:
  /**
   * @brief 控制源枚举
   */
  enum class ControlSource : uint8_t {
    CTRL_SOURCE_RC, /* 遥控器控制源 */
    CTRL_SOURCE_AI, /* AI控制源 */
    CTRL_SOURCE_NUM /* 控制源数量 */
  };

  /**
   * @brief 控制模式枚举
   */
  enum class Mode : uint8_t {
    CMD_OP_CTRL,   /* 操作员控制模式 */
    CMD_AUTO_CTRL, /* 自动控制模式 */
  };

  /**
   * @brief 底盘控制命令结构体
   */
  typedef struct {
    float x; /* X轴方向控制量 */
    float y; /* Y轴方向控制量 */
    float z; /* Z轴方向控制量（旋转） */
  } ChassisCMD;

  /**
   * @brief 云台控制命令结构体
   */
  typedef struct {
    LibXR::CycleValue<float> yaw; /* 偏航角（Yaw angle） */
    LibXR::CycleValue<float> pit; /* 俯仰角（Pitch angle） */
    LibXR::CycleValue<float> rol; /* 翻滚角（Roll angle） */
  } GimbalCMD;

  /**
   * @brief 发射控制命令结构体
   */
  typedef struct {
    bool isfire;
  } LauncherCMD;

  /**
   * @brief 完整控制命令数据结构体
   */
  typedef struct {
    GimbalCMD gimbal;          /* 云台控制命令 */
    ChassisCMD chassis;        /* 底盘控制命令 */
    LauncherCMD launcher;      /* 发射控制命令 */
    bool chassis_online;       /* 底盘在线状态 */
    bool gimbal_online;        /* 云台在线状态 */
    ControlSource ctrl_source; /* 控制源 */
  } Data;

  /**
   * @brief 控制事件ID
   */
  enum {
    CMD_EVENT_LOST_CTRL = 0x13212509 /* 丢失控制事件ID */
  };

  /**
   * @brief 获取当前控制模式
   * @return 当前控制模式
   */
  Mode GetCtrlMode() { return this->mode_; }

  /**
   * @brief 获取CMD模块的事件处理器
   * @return 事件处理器引用
   * @details 用于其他模块绑定事件或激活事件
   */
  LibXR::Event& GetEvent() { return cmd_event_; }

  /**
   * @brief 获取在线状态
   * @return 是否在线
   */
  bool Online() { return this->online_; }

  /**
   * @brief CMD构造函数
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param mode 控制模式，默认为操作员控制模式
   * @param chassis_cmd_topic_name 底盘命令主题名称
   * @param gimbal_cmd_topic_name 云台命令主题名称
   */
  CMD(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, Mode mode,
      const char* chassis_cmd_topic_name, const char* gimbal_cmd_topic_name,
      const char* launcher_cmd_topic_name)
      : mode_(mode),
        data_in_tp_(LibXR::Topic::CreateTopic<Data>("cmd_data_in")),
        chassis_data_tp_(chassis_cmd_topic_name, sizeof(ChassisCMD)),
        gimbal_data_tp_(gimbal_cmd_topic_name, sizeof(GimbalCMD)),
        fire_data_tp_(launcher_cmd_topic_name, sizeof(LauncherCMD)) {
    UNUSED(hw);
    UNUSED(app);
    // 创建事件回调函数，用于处理控制模式切换事件
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, CMD* cmd, uint32_t event_id) {
          UNUSED(in_isr);
          cmd->EventHandler(event_id);
        },
        this);
    // 注册控制模式事件处理回调
    cmd_event_.Register(static_cast<uint32_t>(Mode::CMD_OP_CTRL), callback);
    cmd_event_.Register(static_cast<uint32_t>(Mode::CMD_AUTO_CTRL), callback);
  }

  /**
   * @brief 设置控制模式
   * @param mode 要设置的控制模式
   * @details 根据不同的控制模式配置相应的数据处理回调函数
   */
  void SetCtrlMode(Mode mode) {
    this->mode_ = mode;
    // 操作员控制模式处理逻辑
    if (mode == Mode::CMD_OP_CTRL) {
      // 操作员控制模式下的数据处理函数
      auto op_ctrl_fn = [](bool in_isr, CMD* cmd, LibXR::RawData& raw_data) {
        UNUSED(in_isr);
        UNUSED(raw_data);

        /* 检查在线状态 */
        if (!cmd->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_RC)]
                 .chassis_online &&
            cmd->online_) {
          cmd->cmd_event_.Active(CMD_EVENT_LOST_CTRL);
          cmd->online_ = false;
        } else if (cmd->data_[static_cast<size_t>(
                                  ControlSource::CTRL_SOURCE_RC)]
                       .chassis_online) {
          cmd->online_ = true;
        }

        auto& data_to_publish = cmd->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_RC)];
        cmd->gimbal_data_tp_.Publish(data_to_publish.gimbal);
        cmd->chassis_data_tp_.Publish(data_to_publish.chassis);
        cmd->fire_data_tp_.Publish(data_to_publish.launcher);
      };

      auto op_ctrl_callback =
          LibXR::Callback<LibXR::RawData&>::Create(op_ctrl_fn, this);
      this->data_in_tp_.RegisterCallback(op_ctrl_callback);
    }

    // 自动控制模式处理逻辑
    if (mode == Mode::CMD_AUTO_CTRL) {
      auto auto_ctrl_fn = [](bool in_isr, CMD* cmd, LibXR::RawData& raw_data) {
        UNUSED(in_isr);
        UNUSED(raw_data);

        /* 检查在线状态 */
        if (!cmd->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_RC)]
                 .chassis_online &&
            cmd->online_) {
          cmd->cmd_event_.Active(CMD_EVENT_LOST_CTRL);
          cmd->online_ = false;
        } else if (cmd->data_[static_cast<size_t>(
                                  ControlSource::CTRL_SOURCE_RC)]
                       .chassis_online) {
          cmd->online_ = true;
        }

        /* 根据控制源发布命令：底盘/云台 AI 在线优先，离线用 RC */
        const Data &rc_data =
            cmd->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_RC)];
        const Data &ai_data =
            cmd->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)];

        ChassisCMD out_chassis =
            ai_data.chassis_online ? ai_data.chassis : rc_data.chassis;

        GimbalCMD out_gimbal =
            ai_data.gimbal_online ? ai_data.gimbal : rc_data.gimbal;

        LauncherCMD out_launcher;
        out_launcher.isfire =
            (ai_data.launcher.isfire && rc_data.launcher.isfire);

        cmd->gimbal_data_tp_.Publish(out_gimbal);
        cmd->chassis_data_tp_.Publish(out_chassis);
        cmd->fire_data_tp_.Publish(out_launcher);
      };

      auto auto_ctrl_callback =
          LibXR::Callback<LibXR::RawData&>::Create(auto_ctrl_fn, this);
      this->data_in_tp_.RegisterCallback(auto_ctrl_callback);
    }
  }

  /**
   * @brief 事件处理器
   * @param event_id 事件ID
   * @details 处理来自事件系统的控制模式切换请求
   */
  void EventHandler(uint32_t event_id) {
    SetCtrlMode(static_cast<Mode>(event_id));
  }

  /**
   * @brief 注册控制器
   * @tparam SourceDataType 源数据类型
   * @param source 源主题
   * @details 将外部控制源的数据接入CMD系统，并进行预处理和分发
   */
  template <typename SourceDataType>
  void RegisterController(LibXR::Topic& source) {
    /* 定义链接函数 */
    auto link_fn = [](bool in_isr, CMD* cmd, LibXR::RawData& raw_data) {
      UNUSED(in_isr);

      /* 获取源数据 */
      SourceDataType& source_data =
          *static_cast<SourceDataType*>(raw_data.addr_);

      /* 处理CMD::Data类型数据 */
      if constexpr (std::is_same_v<SourceDataType, CMD::Data>) {
        Data& cmd_data = source_data;
        if (cmd_data.ctrl_source < CMD::ControlSource::CTRL_SOURCE_NUM) {
          /* 存储控制数据 */
          cmd->data_[static_cast<size_t>(cmd_data.ctrl_source)] = cmd_data;

          /* 更新在线状态 */
          if (cmd_data.chassis_online) {
            cmd->online_ = true;
          }
        }
      }

      /* 将数据转发到data_in_tp_主题 */
      cmd->data_in_tp_.Publish(source_data);
    };

    /* 创建回调并注册 */
    auto cb = LibXR::Callback<LibXR::RawData&>::Create(link_fn, this);
    source.RegisterCallback(cb);
  }

  /**
   * @brief 监控函数重写
   */
  void OnMonitor() override {}

 private:
  bool online_ = false;       /* 在线状态 */
  Mode mode_;                 /* 当前控制模式 */
  LibXR::Event cmd_event_;    /* 事件处理器 */
  std::array<Data, static_cast<size_t>(ControlSource::CTRL_SOURCE_NUM)>
      data_{};                      /* 各控制源的数据 */
  LibXR::Topic data_in_tp_;         /* 命令输入主题 */
  LibXR::Topic chassis_data_tp_;    /* 底盘命令主题 */
  LibXR::Topic gimbal_data_tp_;     /* 云台命令主题 */
  LibXR::Topic fire_data_tp_;       /* 开火命令主题 */
  LibXR::Topic host_euler_data_tp_; /* 上位机欧拉角主题 */
};
