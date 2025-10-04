#pragma once

/* clang-format off */
/* === MODULE MANIFEST V2 ===
module_description: 控制命令模块
constructor_args:
  - mode: CMD::Mode::CMD_OP_CTRL
  - chassis_cmd_topic_name: "chassis_cmd"
  - gimbal_cmd_topic_name: "gimbal_cmd"
=== END MANIFEST === */
/* clang-format on */

/**
 * @file CMD.hpp
 * @brief 控制命令处理模块
 * @details 负责处理来自不同控制源的命令，并将其转发到相应的执行单元
 */

#include <vector>
#include "app_framework.hpp"
#include "event.hpp"
#include "message.hpp"
#include "cycle_value.hpp"

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
  typedef enum {
    CTRL_SOURCE_RC, /* 遥控器控制源 */
    CTRL_SOURCE_AI, /* AI控制源 */
    CTRL_SOURCE_NUM /* 控制源数量 */
  } ControlSource;

  /**
   * @brief 控制模式枚举
   */
  typedef enum {
    CMD_OP_CTRL, /* 操作员控制模式 */
    CMD_AUTO_CTRL, /* 自动控制模式 */
  } Mode;

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
   * @brief 完整控制命令数据结构体
   */
  typedef struct {
    GimbalCMD gimbal; /* 云台控制命令 */
    ChassisCMD chassis; /* 底盘控制命令 */
    bool online; /* 在线状态 */
    ControlSource ctrl_source; /* 控制源 */
  } Data;

  /**
   * @brief 控制事件ID
   */
  enum {
    CMD_EVENT_LOST_CTRL = 0x13212509 /* 丢失控制事件ID */
  };

  /**
   * @brief 事件映射项结构体
   */
  typedef struct {
    uint32_t source; /* 源事件ID */
    uint32_t target; /* 目标事件ID */
  } EventMapItem;

  /**
   * @brief 注册事件回调
   * @tparam Type 回调参数类型
   * @tparam EventType 事件类型
   * @param callback 回调函数
   * @param arg 回调参数
   * @param map 事件映射表
   */
  template <typename Type, typename EventType>
  void RegisterEvent(
      void (*callback)(EventType event, Type arg), Type arg,
      const std::vector<EventMapItem>& map) {
    typedef struct {
      uint32_t target_event;
      void (*callback)(EventType event, Type arg);
      void* arg;
    } EventCallbackBlock;

    auto cmd_callback = [](uint32_t event, void* arg) {
      UNUSED(event);
      EventCallbackBlock* block = static_cast<EventCallbackBlock*>(arg);

      block->callback(static_cast<EventType>(block->target_event),
                      static_cast<Type>(block->arg));
    };

    for (const auto& item : map) {
      auto* block = new EventCallbackBlock{
          .target_event = item.target,
          .callback = callback,
          .arg = arg,
      };

      auto cb = LibXR::Callback<uint32_t>::Create(cmd_callback, block);

      this->event_.Register(item.source, cb);
    }
  }

  /**
   * @brief 设置控制源
   * @param source 要设置的控制源
   */
  void SetCtrlSource(ControlSource source) {
    this->ctrl_source_ = source;
  }

  /**
   * @brief 获取当前控制源
   * @return 当前控制源
   */
  ControlSource GetCtrlSource() { return this->ctrl_source_; }

  /**
   * @brief 获取当前控制模式
   * @return 当前控制模式
   */
  Mode GetCtrlMode() { return this->mode_; }

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
  CMD(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
      Mode mode , const char *chassis_cmd_topic_name , const char *gimbal_cmd_topic_name):
    mode_(mode),
    chassis_data_tp_(chassis_cmd_topic_name,sizeof(ChassisCMD)),
    gimbal_data_tp_(gimbal_cmd_topic_name,sizeof(GimbalCMD)) {
    /* 创建主题 */
    data_in_tp_ = LibXR::Topic::CreateTopic<Data>("cmd_data_in");

    /* 操作员控制模式回调函数 */
    auto op_ctrl_fn = [](bool in_isr, CMD* cmd, LibXR::RawData& raw_data) {
      UNUSED(in_isr);
      UNUSED(raw_data);

      /* 检查在线状态 */
      if (!cmd->data_[CTRL_SOURCE_RC].online && cmd->online_) {
        cmd->event_.Active(CMD_EVENT_LOST_CTRL);
        cmd->online_ = false;
      } else if (cmd->data_[CTRL_SOURCE_RC].online) {
        cmd->online_ = true;
      }

      /* 根据控制源发布命令 */
      if (cmd->ctrl_source_ == CTRL_SOURCE_RC ||
          (!cmd->data_[cmd->ctrl_source_].online)) {
        /* 使用遥控器控制源 */
        cmd->gimbal_data_tp_.Publish(cmd->data_[CTRL_SOURCE_RC].gimbal);
        cmd->chassis_data_tp_.Publish(cmd->data_[CTRL_SOURCE_RC].chassis);
      } else if (cmd->ctrl_source_ == CTRL_SOURCE_AI &&
                 cmd->data_[CTRL_SOURCE_AI].online) {
        /* 使用AI控制源（混合模式：AI控制云台，遥控器控制底盘） */
        cmd->gimbal_data_tp_.Publish(cmd->data_[CTRL_SOURCE_AI].gimbal);
        cmd->chassis_data_tp_.Publish(cmd->data_[CTRL_SOURCE_RC].chassis);
      }
    };

    /* 自动控制模式回调函数 */
    auto auto_ctrl_fn = [](bool in_isr, CMD* cmd,
                           LibXR::RawData& raw_data) {
      UNUSED(in_isr);
      UNUSED(raw_data);

      /* 创建新的控制数据 */
      Data new_data;
      new_data.online = true;
      new_data.ctrl_source = CTRL_SOURCE_AI;

      cmd->data_[CTRL_SOURCE_RC] = new_data;

      /* 检查在线状态 */
      if (!cmd->data_[CTRL_SOURCE_RC].online && cmd->online_) {
        cmd->event_.Active(CMD_EVENT_LOST_CTRL);
        cmd->online_ = false;
      } else if (cmd->data_[CTRL_SOURCE_RC].online) {
        cmd->online_ = true;
      }

      /* 根据控制源发布命令 */
      if (cmd->ctrl_source_ == CTRL_SOURCE_RC ||
          (!cmd->data_[cmd->ctrl_source_].online)) {
        cmd->gimbal_data_tp_.Publish(cmd->data_[CTRL_SOURCE_RC].gimbal);
        cmd->chassis_data_tp_.Publish(cmd->data_[CTRL_SOURCE_RC].chassis);
      } else if (cmd->ctrl_source_ == CTRL_SOURCE_AI &&
                 cmd->data_[CTRL_SOURCE_AI].online) {
        cmd->gimbal_data_tp_.Publish(cmd->data_[CTRL_SOURCE_AI].gimbal);
        cmd->chassis_data_tp_.Publish(cmd->data_[CTRL_SOURCE_AI].chassis);
      }
    };

    /* 创建回调对象 */
    auto op_ctrl_callback_ = LibXR::Callback<LibXR::RawData&>::Create(
        op_ctrl_fn, this);
    auto auto_ctrl_callback_ = LibXR::Callback<LibXR::RawData&>::Create(
        auto_ctrl_fn, this);

    /* 根据模式注册回调 */
    switch (this->mode_) {
      case CMD_OP_CTRL:
        this->ctrl_source_ = CTRL_SOURCE_RC;
        this->data_in_tp_.RegisterCallback(op_ctrl_callback_);
        break;
      case CMD_AUTO_CTRL:
        this->data_in_tp_.RegisterCallback(auto_ctrl_callback_);
        break;
    }
  }

  /**
   * @brief 注册控制器
   * @tparam SourceDataType 源数据类型
   * @param source 源主题
   */
  template <typename SourceDataType>
  void RegisterController(LibXR::Topic& source) {
    /* 定义链接函数 */
    auto link_fn = [](bool in_isr, CMD* cmd, LibXR::RawData& raw_data) {
      UNUSED(in_isr);

      /* 获取源数据 */
      SourceDataType& source_data = *static_cast<SourceDataType*>(raw_data.
        addr_);

      /* 处理CMD::Data类型数据 */
      if constexpr (std::is_same_v<SourceDataType, CMD::Data>) {
        Data& cmd_data = source_data;
        if (cmd_data.ctrl_source < CMD::CTRL_SOURCE_NUM) {
          /* 存储控制数据 */
          cmd->data_[cmd_data.ctrl_source] = cmd_data;

          /* 更新在线状态 */
          if (cmd_data.online) {
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
  void OnMonitor() override {
  }

private:
  bool online_ = false; /* 在线状态 */
  ControlSource ctrl_source_; /* 当前控制源 */
  Mode mode_; /* 当前控制模式 */
  LibXR::Event event_; /* 事件处理器 */
  std::array<Data, CTRL_SOURCE_NUM> data_{}; /* 各控制源的数据 */
  LibXR::Topic data_in_tp_; /* 命令输入主题 */
  LibXR::Topic chassis_data_tp_; /* 底盘命令主题 */
  LibXR::Topic gimbal_data_tp_; /* 云台命令主题 */
};
