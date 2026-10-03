#pragma once

/* clang-format off */
/* === MODULE MANIFEST V2 ===
module_description: 控制命令中枢：汇总遥控器与上位机输入，发布底盘、云台、发射命令 / Control command hub that merges remote-controller and host inputs and publishes chassis, gimbal and launcher commands
depends: []
=== END MANIFEST === */
/* clang-format on */

#include <array>
#include <cmath>

#include "event.hpp"
#include "libxr_def.hpp"
#include "message.hpp"

/**
 * @brief 控制命令中枢：汇总遥控器与上位机输入，发布底盘、云台、发射命令。
 *        Control command hub that merges remote-controller and host inputs and publishes
 *        chassis, gimbal and launcher commands.
 */
class CMD
{
 public:
  /**
   * @brief 控制源。
   *        Control source.
   */
  enum class ControlSource : uint8_t
  {
    CTRL_SOURCE_RC,  ///< 遥控器 Remote controller
    CTRL_SOURCE_AI,  ///< 上位机 / 自动控制 Host / automatic control
    CTRL_SOURCE_NUM  ///< 控制源数量 Number of control sources
  };

  /**
   * @brief 遥控链路输入源。
   *        Remote-controller input source.
   */
  enum class RCInputSource : uint8_t
  {
    RC_INPUT_DR16,  ///< DR16 遥控接收机 DR16 receiver
    RC_INPUT_VT13,  ///< VT13 图传链路遥控 VT13 video-link remote controller
    RC_INPUT_NUM    ///< 输入源数量 Number of input sources
  };

  /**
   * @brief 控制模式。
   *        Control mode.
   */
  enum class Mode : uint8_t
  {
    CMD_OP_CTRL,   ///< 操作手控制 Operator control
    CMD_AUTO_CTRL,  ///< 自动控制 Automatic control
  };

  /**
   * @brief 底盘自定义模式。
   *        Chassis custom mode.
   */
  enum class ChasStat : int8_t
  {
    NONE = 0,     ///< 无模式 No mode
    BOOST = 1,    ///< 加速 Boost
    STRETCH = 2,  ///< 伸腿 Leg stretch
  };

  /**
   * @brief 底盘命令。
   *        Chassis command.
   */
  typedef struct
  {
    float x;  ///< X 轴方向控制量 Control value along the X axis
    float y;  ///< Y 轴方向控制量 Control value along the Y axis
    float z;  ///< Z 轴方向控制量（旋转）Control value around the Z axis (rotation)
    ChasStat self_define;  ///< 自定义模式 Custom mode
  } ChassisCMD;

  /**
   * @brief 云台命令。
   *        Gimbal command.
   */
  typedef struct
  {
    float yaw;       ///< 偏航角 Yaw angle
    float pit;       ///< 俯仰角 Pitch angle
    float rol;       ///< 翻滚角 Roll angle
    float yaw_dot;   ///< yaw 角速度 Yaw angular velocity
    float yaw_ddot;  ///< yaw 角加速度 Yaw angular acceleration
    float pit_dot;   ///< pit 角速度 Pitch angular velocity
    float pit_ddot;  ///< pit 角加速度 Pitch angular acceleration
    float rol_dot;   ///< rol 角速度 Roll angular velocity
    float rol_ddot;  ///< rol 角加速度 Roll angular acceleration
  } GimbalCMD;

  /**
   * @brief 发射命令。
   *        Launcher command.
   */
  typedef struct
  {
    bool isfire;  ///< 是否开火 Whether to fire
  } LauncherCMD;

  /**
   * @brief 一个控制源的完整命令数据。
   *        Complete command data of one control source.
   */
  typedef struct
  {
    GimbalCMD gimbal;      ///< 云台命令 Gimbal command
    ChassisCMD chassis;    ///< 底盘命令 Chassis command
    LauncherCMD launcher;  ///< 发射命令 Launcher command
    bool chassis_online;  ///< 底盘命令有效 Chassis command valid (online)
    bool gimbal_online;   ///< 云台命令有效 Gimbal command valid (online)
    ControlSource ctrl_source;  ///< 控制源 Control source
  } Data;

  /**
   * @brief CMD 事件 ID。
   *        CMD event IDs.
   */
  enum
  {
    CMD_EVENT_START_CTRL = 0x13212508,  ///< 遥控器上线 Remote controller online
    CMD_EVENT_LOST_CTRL = 0x13212509    ///< 遥控器离线 Remote controller offline
  };

  /**
   * @brief 获取当前控制模式。
   *        Get the current control mode.
   *
   * @return 当前控制模式。
   *         Current control mode.
   */
  Mode GetCtrlMode() { return this->mode_; }

  /**
   * @brief 获取 AI 数据的云台在线状态。
   *        Get the gimbal online state of the AI data.
   *
   * @return AI 数据的 gimbal_online。
   *         The gimbal_online of the AI data.
   */
  bool GetAIGimbalStatus()
  {
    return this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)].gimbal_online;
  }

  /**
   * @brief 获取 CMD 的事件对象。
   *        Get the event object of CMD.
   *
   * @return 事件对象引用，用于绑定事件或激活事件；激活 Mode 的数值即切换控制模式。
   *         Event object reference for binding or activating events; activating the value
   *         of a Mode switches the control mode.
   */
  LibXR::Event& GetEvent() { return cmd_event_; }

  /**
   * @brief 获取遥控器在线状态。
   *        Get the remote-controller online state.
   *
   * @return 遥控器是否在线。
   *         Whether the remote controller is online.
   */
  bool Online() { return this->online_; }

  /**
   * @brief 以 DR16 输入源写入遥控器数据，并发布命令。
   *        Write remote-controller data with the DR16 input source and publish the
   *        commands.
   *
   * @param rc_data 遥控器数据。
   *                Remote-controller data.
   */
  void FeedRC(const Data& rc_data)
  {
    this->FeedRC(RCInputSource::RC_INPUT_DR16, rc_data);
  }

  /**
   * @brief 按遥控输入源写入遥控器数据，并发布命令。
   *        Write remote-controller data of the given input source and publish the
   *        commands.
   *
   * @param source 遥控输入源，超出范围时忽略本次写入。
   *               Remote-controller input source; the write is ignored when out of range.
   * @param rc_data 遥控器数据。
   *                Remote-controller data.
   */
  void FeedRC(RCInputSource source, const Data& rc_data)
  {
    const auto source_index = static_cast<size_t>(source);
    if (source_index >= static_cast<size_t>(RCInputSource::RC_INPUT_NUM))
    {
      return;
    }

    this->rc_input_data_[source_index] = rc_data;
    this->rc_input_seq_[source_index] = ++this->rc_update_seq_;

    if (rc_data.chassis_online && this->IsRCInputActive(rc_data))
    {
      this->active_rc_input_ = source;
    }

    this->ProcessAndPublish();
  }

  /**
   * @brief 写入 AI 控制数据，并发布命令。
   *        Write AI control data and publish the commands.
   *
   * @param ai_data AI 控制数据。
   *                AI control data.
   */
  void FeedAI(const Data& ai_data)
  {
    this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)] = ai_data;
    this->ProcessAndPublish();
  }

  /**
   * @brief 构造 CMD，创建三路命令 Topic 并注册控制模式事件。
   *        Construct CMD, create the three command Topics and register the control mode
   *        events.
   *
   * @param mode 初始控制模式，默认为操作手控制。
   *             Initial control mode, operator control by default.
   * @param chassis_cmd_topic_name 底盘命令 Topic 名称。
   *                               Name of the chassis command Topic.
   * @param gimbal_cmd_topic_name 云台命令 Topic 名称。
   *                              Name of the gimbal command Topic.
   * @param launcher_cmd_topic_name 发射命令 Topic 名称。
   *                                Name of the launcher command Topic.
   */
  CMD(Mode mode = CMD::Mode::CMD_OP_CTRL,
      const char* chassis_cmd_topic_name = "chassis_cmd",
      const char* gimbal_cmd_topic_name = "gimbal_cmd",
      const char* launcher_cmd_topic_name = "launcher_cmd")
      : mode_(mode),
        chassis_data_tp_(
            LibXR::Topic::CreateTopic<ChassisCMD>(chassis_cmd_topic_name, nullptr, true)),
        gimbal_data_tp_(
            LibXR::Topic::CreateTopic<GimbalCMD>(gimbal_cmd_topic_name, nullptr, true)),
        fire_data_tp_(LibXR::Topic::CreateTopic<LauncherCMD>(launcher_cmd_topic_name,
                                                             nullptr, true))
  {
    // 创建事件回调函数
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, CMD* cmd, uint32_t event_id)
        {
          UNUSED(in_isr);
          cmd->EventHandler(event_id);
        },
        this);
    // 注册控制模式事件处理回调
    this->cmd_event_.Register(static_cast<uint32_t>(Mode::CMD_OP_CTRL), callback);
    this->cmd_event_.Register(static_cast<uint32_t>(Mode::CMD_AUTO_CTRL), callback);
  }

  /**
   * @brief 设置控制模式。
   *        Set the control mode.
   *
   * @param mode 控制模式。
   *             Control mode.
   */
  void SetCtrlMode(Mode mode) { this->mode_ = mode; }

  /**
   * @brief 处理控制模式事件，把事件 ID 作为控制模式。
   *        Handle a control mode event, taking the event ID as the control mode.
   *
   * @param event_id 事件 ID，取值为 Mode 的数值。
   *                 Event ID, the numeric value of a Mode.
   */
  void EventHandler(uint32_t event_id) { this->SetCtrlMode(static_cast<Mode>(event_id)); }

  /**
   * @brief 保留的控制源注册接口，函数体为空。
   *        Reserved control source registration interface with an empty body.
   *
   * @tparam SourceDataType 源数据类型。
   *                        Source data type.
   * @param source 源 Topic。
   *               Source Topic.
   */
  template <typename SourceDataType>
  void RegisterController(LibXR::Topic& source)
  {
    UNUSED(source);
  }

 private:
  bool online_ = false;     ///< 遥控器在线状态 Remote-controller online state
  Mode mode_;               ///< 当前控制模式 Current control mode
  LibXR::Event cmd_event_;  ///< CMD 事件 CMD event
  std::array<Data, static_cast<size_t>(ControlSource::CTRL_SOURCE_NUM)>
      data_{};  ///< 各控制源的数据 Data of each control source
  std::array<Data, static_cast<size_t>(RCInputSource::RC_INPUT_NUM)>
      rc_input_data_{};  ///< 各遥控输入源的数据 Data of each remote-controller input source
  std::array<uint32_t, static_cast<size_t>(RCInputSource::RC_INPUT_NUM)>
      rc_input_seq_{};  ///< 各遥控输入源的数据序号 Sequence number of each RC input source
  LibXR::Topic chassis_data_tp_;  ///< 底盘命令 Topic Chassis command Topic
  LibXR::Topic gimbal_data_tp_;  ///< 云台命令 Topic Gimbal command Topic
  LibXR::Topic fire_data_tp_;  ///< 发射命令 Topic Launcher command Topic
  LibXR::Topic host_euler_data_tp_;  ///< 上位机欧拉角 Topic Host Euler angle Topic
  RCInputSource active_rc_input_ =
      RCInputSource::RC_INPUT_DR16;  ///< 当前活动遥控输入源 Active remote-controller input source
  uint32_t rc_update_seq_ = 0;  ///< 遥控输入数据更新序号 Remote-controller update sequence number

  static bool IsRCInputOnline(const Data& rc_data) { return rc_data.chassis_online; }

  static bool IsRCInputActive(const Data& rc_data)
  {
    constexpr float RC_ACTIVITY_EPS = 0.05f;

    return std::fabs(rc_data.chassis.x) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.chassis.y) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.chassis.z) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.gimbal.yaw) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.gimbal.pit) > RC_ACTIVITY_EPS ||
           std::fabs(rc_data.gimbal.rol) > RC_ACTIVITY_EPS ||
           (rc_data.chassis.self_define != ChasStat::NONE) || rc_data.launcher.isfire;
  }

  static Data MakeOfflineRCData()
  {
    Data rc_data{};
    rc_data.chassis_online = false;
    rc_data.gimbal_online = false;
    rc_data.ctrl_source = ControlSource::CTRL_SOURCE_RC;
    return rc_data;
  }

  Data SelectRCData()
  {
    const auto dr16_index = static_cast<size_t>(RCInputSource::RC_INPUT_DR16);
    const auto vt13_index = static_cast<size_t>(RCInputSource::RC_INPUT_VT13);
    const auto active_index = static_cast<size_t>(this->active_rc_input_);

    // 当前活动源在线则持续使用
    if (active_index < static_cast<size_t>(RCInputSource::RC_INPUT_NUM) &&
        this->IsRCInputOnline(this->rc_input_data_[active_index]))
    {
      return this->rc_input_data_[active_index];
    }

    // 活动源离线后切换
    if (this->active_rc_input_ == RCInputSource::RC_INPUT_DR16)
    {
      if (this->IsRCInputOnline(this->rc_input_data_[vt13_index]))
      {
        this->active_rc_input_ = RCInputSource::RC_INPUT_VT13;
        return this->rc_input_data_[vt13_index];
      }
      if (this->IsRCInputOnline(this->rc_input_data_[dr16_index]))
      {
        this->active_rc_input_ = RCInputSource::RC_INPUT_DR16;
        return this->rc_input_data_[dr16_index];
      }
    }
    else
    {
      if (this->IsRCInputOnline(this->rc_input_data_[dr16_index]))
      {
        this->active_rc_input_ = RCInputSource::RC_INPUT_DR16;
        return this->rc_input_data_[dr16_index];
      }
      if (this->IsRCInputOnline(this->rc_input_data_[vt13_index]))
      {
        this->active_rc_input_ = RCInputSource::RC_INPUT_VT13;
        return this->rc_input_data_[vt13_index];
      }
    }

    return MakeOfflineRCData();
  }

  void ProcessAndPublish()
  {
    const Data rc_data = this->SelectRCData();
    const Data& ai_data = this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_AI)];

    this->data_[static_cast<size_t>(ControlSource::CTRL_SOURCE_RC)] = rc_data;

    if (!rc_data.chassis_online && this->online_)
    {
      this->cmd_event_.Active(CMD_EVENT_LOST_CTRL);
      this->online_ = false;
    }
    else if (rc_data.chassis_online && !this->online_)
    {
      this->cmd_event_.Active(CMD_EVENT_START_CTRL);
      this->online_ = true;
    }

    if (this->mode_ == Mode::CMD_OP_CTRL)
    {
      Data out = rc_data;
      this->gimbal_data_tp_.Publish(out.gimbal);
      this->chassis_data_tp_.Publish(out.chassis);
      this->fire_data_tp_.Publish(out.launcher);
    }
    else
    {
      ChassisCMD out_chassis = ai_data.chassis_online ? ai_data.chassis : rc_data.chassis;
      GimbalCMD out_gimbal = ai_data.gimbal_online ? ai_data.gimbal : rc_data.gimbal;
      LauncherCMD out_launcher;
      out_launcher.isfire = (ai_data.launcher.isfire && rc_data.launcher.isfire);

      ChassisCMD chassis = out_chassis;
      GimbalCMD gimbal = out_gimbal;
      LauncherCMD launcher = out_launcher;
      this->gimbal_data_tp_.Publish(gimbal);
      this->chassis_data_tp_.Publish(chassis);
      this->fire_data_tp_.Publish(launcher);
    }
  }
};
