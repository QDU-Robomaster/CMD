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

#include <array>
#include <cmath>

#include "app_framework.hpp"
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
   * @brief 遥控链路输入源枚举
   */
  enum class RCInputSource : uint8_t {
    RC_INPUT_DR16,
    RC_INPUT_VT13,
    RC_INPUT_NUM
  };

  /**
   * @brief 控制模式枚举
   */
  enum class Mode : uint8_t {
    CMD_OP_CTRL,   /* 操作员控制模式 */
    CMD_AUTO_CTRL, /* 自动控制模式 */
  };

  /**
   * @brief 底盘小模式枚举
   */
  enum class ChasStat : int8_t {
    NONE = 0,    /*无模式*/
    BOOST = 1,   /*加速*/
    STRETCH = 2, /*伸腿*/
  };

  /**
   * @brief 底盘控制命令结构体
   */
  typedef struct {
    float x;              /* X轴方向控制量 */
    float y;              /* Y轴方向控制量 */
    float z;              /* Z轴方向控制量（旋转） */
    ChasStat self_define; /* 自定义按钮 */
  } ChassisCMD;

  /**
   * @brief 云台控制命令结构体
   */
  typedef struct {
    float yaw;      /* 偏航角（Yaw angle） */
    float pit;      /* 俯仰角（Pitch angle） */
    float rol;      /* 翻滚角（Roll angle） */
    float yaw_dot;  /* yaw角速度 */
    float yaw_ddot; /* yaw角加速度 */
    float pit_dot;  /* pit角速度 */
    float pit_ddot; /* pit角加速度 */
    float rol_dot;  /* roll角速度 */
    float rol_ddot; /* roll角加速度 */
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
    CMD_EVENT_START_CTRL = 0x13212508, /* 开始控制事件ID */
    CMD_EVENT_LOST_CTRL = 0x13212509   /* 丢失控制事件ID */
  };

  /**
   * @brief 获取当前控制模式
   * @return 当前控制模式
   */
  Mode GetCtrlMode() { return this->mode_; }

  bool GetAIGimbalStatus() {
    return this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)]
        .gimbal_online;
  }

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
   * @brief 直接写入遥控器控制数据
   * @details 兼容接口，默认按DR16输入源写入
   */
  void FeedRC(const Data& rc_data) {
    this->FeedRC(RCInputSource::RC_INPUT_DR16, rc_data);
  }

  /**
   * @brief 按遥控输入源写入控制数据
   */
  void FeedRC(RCInputSource source, const Data& rc_data) {
    const auto source_index = static_cast<size_t>(source);
    if (source_index >= static_cast<size_t>(RCInputSource::RC_INPUT_NUM)) {
      return;
    }

    this->rc_input_data_[source_index] = rc_data;
    this->rc_input_seq_[source_index] = ++this->rc_update_seq_;

    if (rc_data.chassis_online && this->IsRCInputActive(rc_data)) {
      this->active_rc_input_ = source;
    }

    this->ProcessAndPublish();
  }

  /**
   * @brief 直接写入 AI 控制数据
   */
  void FeedAI(const Data& ai_data) {
    this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)] = ai_data;
    this->ProcessAndPublish();
  }

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
        chassis_data_tp_(chassis_cmd_topic_name, sizeof(ChassisCMD), nullptr,
                         true),
        gimbal_data_tp_(gimbal_cmd_topic_name, sizeof(GimbalCMD), nullptr,
                        true),
        fire_data_tp_(launcher_cmd_topic_name, sizeof(LauncherCMD), nullptr,
                      true) {
    UNUSED(hw);
    UNUSED(app);
    /* 创建事件回调函数 */
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, CMD* cmd, uint32_t event_id) {
          UNUSED(in_isr);
          cmd->EventHandler(event_id);
        },
        this);
    /* 注册控制模式事件处理回调 */
    this->cmd_event_.Register(static_cast<uint32_t>(Mode::CMD_OP_CTRL),
                              callback);
    this->cmd_event_.Register(static_cast<uint32_t>(Mode::CMD_AUTO_CTRL),
                              callback);
  }

  /**
   * @brief 设置控制模式
   * @param mode 要设置的控制模式
   * @details 根据不同的控制模式配置相应的数据处理回调函数
   */
  void SetCtrlMode(Mode mode) { this->mode_ = mode; }

  /**
   * @brief 事件处理器
   * @param event_id 事件ID
   * @details 处理来自事件系统的控制模式切换请求
   */
  void EventHandler(uint32_t event_id) {
    this->SetCtrlMode(static_cast<Mode>(event_id));
  }

  /**
   * @brief 注册控制器
   * @tparam SourceDataType 源数据类型
   * @param source 源主题
   * @details 将外部控制源的数据接入CMD系统，并进行预处理和分发
   */
  template <typename SourceDataType>
  void RegisterController(LibXR::Topic& source) {
    UNUSED(source);
  }

  /**
   * @brief 监控函数重写
   */
  void OnMonitor() override {}

 private:
  bool online_ = false;    /* 在线状态 */
  Mode mode_;              /* 当前控制模式 */
  LibXR::Event cmd_event_; /* 事件处理器 */
  std::array<Data, static_cast<size_t>(ControlSource::CTRL_SOURCE_NUM)>
      data_{}; /* 各控制源的数据 */
  std::array<Data, static_cast<size_t>(RCInputSource::RC_INPUT_NUM)>
      rc_input_data_{}; /* 各遥控输入源的数据 */
  std::array<uint32_t, static_cast<size_t>(RCInputSource::RC_INPUT_NUM)>
      rc_input_seq_{};              /* 各遥控输入源的数据序号 */
  LibXR::Topic chassis_data_tp_;    /* 底盘命令主题 */
  LibXR::Topic gimbal_data_tp_;     /* 云台命令主题 */
  LibXR::Topic fire_data_tp_;       /* 开火命令主题 */
  LibXR::Topic host_euler_data_tp_; /* 上位机欧拉角主题 */
  RCInputSource active_rc_input_ =
      RCInputSource::RC_INPUT_DR16; /* 当前活动遥控输入源 */
  uint32_t rc_update_seq_ = 0;      /* 遥控输入数据更新序号 */

  /*--------------------------工具函数-------------------------------------------------*/
  static bool IsRCInputOnline(const Data& rc_data) {
    return rc_data.chassis_online;
  }

  static bool IsRCInputActive(const Data& rc_data) {
    constexpr float RC_ACTIVITY_EPS = 0.05f;

    return std::fabs(rc_data.chassis.x) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.chassis.y) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.chassis.z) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.gimbal.yaw) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.gimbal.pit) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.gimbal.rol) > RC_ACTIVITY_EPS ||
           (rc_data.chassis.self_define != ChasStat::NONE) ||
           rc_data.launcher.isfire;
  }

  static Data MakeOfflineRCData() {
    Data rc_data{};
    rc_data.chassis_online = false;
    rc_data.gimbal_online = false;
    rc_data.ctrl_source = ControlSource::CTRL_SOURCE_RC;
    return rc_data;
  }

  Data SelectRCData() {
    const auto dr16_index = static_cast<size_t>(RCInputSource::RC_INPUT_DR16);
    const auto vt13_index = static_cast<size_t>(RCInputSource::RC_INPUT_VT13);
    const auto active_index = static_cast<size_t>(this->active_rc_input_);

    /* 当前活动源在线则持续使用 */
    if (active_index < static_cast<size_t>(RCInputSource::RC_INPUT_NUM) &&
        this->IsRCInputOnline(this->rc_input_data_[active_index])) {
      return this->rc_input_data_[active_index];
    }

    /* 活动源离线后切换 */
    if (this->active_rc_input_ == RCInputSource::RC_INPUT_DR16) {
      if (this->IsRCInputOnline(this->rc_input_data_[vt13_index])) {
        this->active_rc_input_ = RCInputSource::RC_INPUT_VT13;
        return this->rc_input_data_[vt13_index];
      }
      if (this->IsRCInputOnline(this->rc_input_data_[dr16_index])) {
        this->active_rc_input_ = RCInputSource::RC_INPUT_DR16;
        return this->rc_input_data_[dr16_index];
      }
    } else {
      if (this->IsRCInputOnline(this->rc_input_data_[dr16_index])) {
        this->active_rc_input_ = RCInputSource::RC_INPUT_DR16;
        return this->rc_input_data_[dr16_index];
      }
      if (this->IsRCInputOnline(this->rc_input_data_[vt13_index])) {
        this->active_rc_input_ = RCInputSource::RC_INPUT_VT13;
        return this->rc_input_data_[vt13_index];
      }
    }

    return MakeOfflineRCData();
  }

  void ProcessAndPublish() {
    const Data rc_data = this->SelectRCData();
    const Data& ai_data =
        this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)];

    this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_RC)] = rc_data;

    if (!rc_data.chassis_online && this->online_) {
      this->cmd_event_.Active(CMD_EVENT_LOST_CTRL);
      this->online_ = false;
    } else if (rc_data.chassis_online && !this->online_) {
      this->cmd_event_.Active(CMD_EVENT_START_CTRL);
      this->online_ = true;
    }

    if (this->mode_ == Mode::CMD_OP_CTRL) {
      Data out = rc_data;
      this->gimbal_data_tp_.Publish(out.gimbal);
      this->chassis_data_tp_.Publish(out.chassis);
      this->fire_data_tp_.Publish(out.launcher);
    } else {
      /* CMD_AUTO_CTRL */
      ChassisCMD out_chassis =
          ai_data.chassis_online ? ai_data.chassis : rc_data.chassis;
      GimbalCMD out_gimbal =
          ai_data.gimbal_online ? ai_data.gimbal : rc_data.gimbal;
      LauncherCMD out_launcher;
      out_launcher.isfire =
          (ai_data.launcher.isfire && rc_data.launcher.isfire);

      ChassisCMD chassis = out_chassis;
      GimbalCMD gimbal = out_gimbal;
      LauncherCMD launcher = out_launcher;
      this->gimbal_data_tp_.Publish(gimbal);
      this->chassis_data_tp_.Publish(chassis);
      this->fire_data_tp_.Publish(launcher);
    }
  }
};
