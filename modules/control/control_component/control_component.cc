/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License atis_control_test_mode
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/control/control_component/control_component.h"

#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/latency_recorder/latency_recorder.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/control_component/common/control_gflags.h"

#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <linux/joystick.h>

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::cyber::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

const double kDoubleEpsilon = 1e-6;

/***********个人添加**************/
void PidControl::SetPIDParam(const PIDParam &pidparam) {
   kp_ = pidparam.kp_;
   ki_ = pidparam.ki_;
   kd_ = pidparam.kd_;
   integrator_limit_level_ = pidparam.integrator_limit_level_;
   output_limit_level_ = pidparam.output_limit_level_;
   is_steer_ = pidparam.is_steer_;
}

void PidControl::Init(const PIDParam &pidparam) {
    first_hit_ = true;
    kp_ = pidparam.kp_;
    ki_ = pidparam.ki_;
    kd_ = pidparam.kd_;
    integral_        = 0.0;
    previous_error_  = 0.0;
    previous_output_ = 0.0;
    integrator_limit_level_ = pidparam.integrator_limit_level_;
    output_limit_level_ = pidparam.output_limit_level_;
    is_steer_ = pidparam.is_steer_;
}
void PidControl::Reset() {
    integral_        = 0.0;
    previous_error_  = 0.0;
    previous_output_ = 0.0;
    first_hit_ = true;
}
double PidControl::ComputePID(const double error, double dt, const double min) {
  if (dt <= 0.0) {
    AWARN << "dt <= 0, will use the last output, dt: " << dt;
    return previous_output_;
  }

  double diff = 0.0;
  double output = 0.0;
  
  if (first_hit_) {
    first_hit_ = false;
  } else {
    diff = (error - previous_error_) / dt;
  }
  integral_ += error * dt * ki_;
  
  if (integral_ > integrator_limit_level_) {
    integral_ = integrator_limit_level_;
  } else if (integral_ < -integrator_limit_level_) {
    integral_ = -integrator_limit_level_;
  }
  
  
  output = error * kp_ + integral_ + diff * kd_;
  if (!is_steer_) {
    if (fabs(output)< 27.0) {
      if (output < 0) {output = -27.0;}
      if (output > 0) {output = 27.0;}
    }
    if (output > output_limit_level_) {
      output = output_limit_level_;
    }
    if (output < -output_limit_level_) {
      output = -output_limit_level_;
    }
  } else {
    if(fabs(output) < 1) {
      if(output < 0){output = -1;}
      if(output > 0){output = 1;}
    }
    if (output > output_limit_level_) {
      output = output_limit_level_;
    }
    if (output < -output_limit_level_) {
      output = -output_limit_level_;
    }
  }

  if (fabs(error) <= min) {
    previous_error_ = 0.0;
    output = 0.0;
  }
  previous_error_ = error;
  previous_output_ = output;
  return output;
}

ControlComponent::ControlComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

bool ControlComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();
  init_time_ = Clock::Now();

  AINFO << "Control init, starting ...";

  ACHECK(
      cyber::common::GetProtoFromFile(FLAGS_pipeline_file, &control_pipeline_))
      << "Unable to load control pipeline file: " + FLAGS_pipeline_file;

  AINFO << "ControlTask pipeline config file: " << FLAGS_pipeline_file
        << " is loaded.";

  // initial controller agent when not using control submodules
  ADEBUG << "FLAGS_use_control_submodules: " << FLAGS_use_control_submodules;
  // if (!FLAGS_is_control_ut_test_mode) {
    if (!FLAGS_use_control_submodules &&
        !control_task_agent_.Init(injector_, control_pipeline_).ok()) {
      // set controller
      ADEBUG << "original control";
      monitor_logger_buffer_.ERROR(
          "Control init controller failed! Stopping...");
      return false;
    }
  // }

  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = FLAGS_chassis_pending_queue_size;

  chassis_reader_ =
      node_->CreateReader<Chassis>(chassis_reader_config, nullptr);
  ACHECK(chassis_reader_ != nullptr);

  cyber::ReaderConfig planning_reader_config;
  planning_reader_config.channel_name = FLAGS_planning_trajectory_topic;
  planning_reader_config.pending_queue_size = FLAGS_planning_pending_queue_size;

  trajectory_reader_ =
      node_->CreateReader<ADCTrajectory>(planning_reader_config, nullptr);
  ACHECK(trajectory_reader_ != nullptr);

  cyber::ReaderConfig planning_command_status_reader_config;
  planning_command_status_reader_config.channel_name =
      FLAGS_planning_command_status;
  planning_command_status_reader_config.pending_queue_size =
      FLAGS_planning_status_msg_pending_queue_size;
  planning_command_status_reader_ =
      node_->CreateReader<external_command::CommandStatus>(
          planning_command_status_reader_config, nullptr);
  ACHECK(planning_command_status_reader_ != nullptr);

  cyber::ReaderConfig localization_reader_config;
  localization_reader_config.channel_name = FLAGS_localization_topic;
  localization_reader_config.pending_queue_size =
      FLAGS_localization_pending_queue_size;

  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      localization_reader_config, nullptr);
  ACHECK(localization_reader_ != nullptr);

  cyber::ReaderConfig pad_msg_reader_config;
  pad_msg_reader_config.channel_name = FLAGS_pad_topic;
  pad_msg_reader_config.pending_queue_size = FLAGS_pad_msg_pending_queue_size;

  pad_msg_reader_ =
      node_->CreateReader<PadMessage>(pad_msg_reader_config, nullptr);
  ACHECK(pad_msg_reader_ != nullptr);

  if (!FLAGS_use_control_submodules) {
    control_cmd_writer_ =
        node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
    ACHECK(control_cmd_writer_ != nullptr);
  } else {
    local_view_writer_ =
        node_->CreateWriter<LocalView>(FLAGS_control_local_view_topic);
    ACHECK(local_view_writer_ != nullptr);
  }

  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // should init_vehicle first, let car enter work status, then use status msg
  // trigger control

  AINFO << "Control default driving action is "
        << DrivingAction_Name((enum DrivingAction)FLAGS_action);
  pad_msg_.set_action((enum DrivingAction)FLAGS_action);

  thread_.reset(new std::thread([this] { CheckJoy(); }));
  PIDParam steer_pid_param_(0.86, 0.14, 0.02, 10.0, 50.0, true);
  // PIDParam throttle_pid_param_(75.0, 2.0, 0.5, 3.0, 65.0, false);
  // PIDParam brake_pid_param_(95.0, 2.5, 0.5, 3.0, 80.0, false);
  PIDParam throttle_pid_param_(4.0, 1.45, 0.0, 10.0, 65.0, false);
  PIDParam brake_pid_param_(19.75, 0.25, 0.0, 10.0, 80.0, false);

  steer_pidcontrol_.Init(steer_pid_param_);
  brake_pidcontrol_.Init(brake_pid_param_);
  throttle_pidcontrol_.Init(throttle_pid_param_);
  is_first = true;

  return true;
}

//个人添加
void ControlComponent::set_terminal_echo(bool enabled) {
    struct termios tty;
    tcgetattr(STDIN_FILENO, &tty);
    if (enabled) {
        tty.c_lflag |= ECHO;
    } else {
        tty.c_lflag &= ~ECHO;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &tty);
}

void ControlComponent::CheckJoy() {
    const char* inputDevPath = "/dev/input/event7";  //终端使用evtest查看  
    int inputDev;
    
    inputDev = open(inputDevPath, O_RDONLY);
    if((inputDev == -1)) {
      printf("设备打开失败\n");
      return;
    }
    printf("设备打开成功\n");
    set_terminal_echo(false);
    struct input_event inputEvent;
    while(1){
      ssize_t bytesRead = read(inputDev, &inputEvent, sizeof(inputEvent));
      if(inputEvent.type == EV_ABS){
        if(inputEvent.code == ABS_RZ){
          if ( inputEvent.value <= 15 ){
            key_up_ =false;
            thro_target_value_ = 2.0;
          } else {
            key_up_ = true;
            key_down_ = false;
            thro_target_value_ = 1.65 - (double)(inputEvent.value - 15) / 240;
          }
        }
        if(inputEvent.code == ABS_Z){
          if(inputEvent.value <= 15){
            key_down_ = false;
            brake_target_value_ = 2.5;
          }else {
            key_down_ = true;
            key_up_ = false;
            brake_target_value_ = 1.8 - (double)(inputEvent.value - 15) / 240;
          }
        }
        if(inputEvent.code == ABS_HAT0X){
          if(inputEvent.value == -1){
            key_left_ = true;
            key_right_ = false;
          } else if(inputEvent.value == 1){
            key_right_ = true;
            key_left_ = false;
          } else {
            key_right_ = false;
            key_left_ = false;
          }

        }
        if(inputEvent.code == ABS_HAT0Y){
          if(inputEvent.value == -1){
            //steer_pidcontrol_.SetD(0.01);
            // throttle_pidcontrol_.SetD(0.01);
            // brake_pidcontrol_.SetD(0.01);
            scale += 0.01;
          } else if(inputEvent.value == 1){
            scale -= 0.01;
      
            //steer_pidcontrol_.SetD(-0.01);
            // throttle_pidcontrol_.SetD(-0.01);
            // brake_pidcontrol_.SetD(-0.01);
          }
        }
      }
      if(inputEvent.type == EV_KEY){
        if(inputEvent.code == BTN_SOUTH){
          if(inputEvent.value == 1){
            is_switch = true;
          }
        }
        if(inputEvent.code == BTN_EAST){
          if(inputEvent.value == 1){
            is_switch = false;
          }
        }
        if(inputEvent.code == BTN_NORTH){
          if(inputEvent.value == 1){
            //steer_pidcontrol_.SetP(0.01);
            // throttle_pidcontrol_.SetP(0.01);
            // brake_pidcontrol_.SetP(0.01);
            
          }
        }
        if(inputEvent.code == BTN_WEST){
          if(inputEvent.value == 1){
            //steer_pidcontrol_.SetP(-0.01);
            // throttle_pidcontrol_.SetP(-0.01);
            // brake_pidcontrol_.SetP(-0.01);
          }
        }
        if(inputEvent.code == BTN_TR){
          if(inputEvent.value == 1){
            //steer_pidcontrol_.SetI(0.01);
            // throttle_pidcontrol_.SetI(0.01);
            // brake_pidcontrol_.SetI(0.01);
          }
        }
        if(inputEvent.code == BTN_TL){
          if(inputEvent.value == 1){
            //steer_pidcontrol_.SetI(-0.01);
            // brake_pidcontrol_.SetI(-0.01);
            // throttle_pidcontrol_.SetI(-0.01);
          }
        }
      }
     usleep(30000);
    }   
}

void ControlComponent::OnKeyBoard(double &cur_brake_v, double &cur_thro_v, 
  double &cur_steer_angle, ControlCommand &control_command) {

  double error = 0.0;
  double output = 0.0;
  if(is_first) {
      steer_target_value_ = cur_steer_angle;
      is_first = false;
  }

  if(key_left_ && !key_right_) {
    wheel_angle_ = -1;
    is_steer_control_ = true;
  } else if(key_right_ && !key_left_) {
    wheel_angle_ = 1;
    is_steer_control_ = true;
  } else {
    wheel_angle_ = 0.0;
    is_steer_control_ = false;
  }

  if (is_steer_control_) {
    steer_target_value_ = wheel_angle_*18;
    steer_target_value_ += cur_steer_angle;
    if (steer_target_value_ > 720.0) {
      steer_target_value_ = 720.0;
    } else if(steer_target_value_ < -720){
      steer_target_value_ = -720;
    }
    is_steer_control_ = false;
  }
  error = steer_target_value_ - cur_steer_angle;
  if (abs(error) >  0.001) {
    printf("normal\n");
  } 
  output = steer_pidcontrol_.ComputePID(error, 0.01, 2.0);
  control_command.set_steering_rate(output);

  printf("[手动]>>>>>方向盘 目标转角：%.3f;  当前转角：%.3f;\n", steer_target_value_, cur_steer_angle);
  printf("[手动]>>>>>方向盘 目标差值:%.3f\n", error);
  printf("[手动]>>>>>方向盘 输   出:%.3f\n", output);
  
  
  if(key_up_&&!key_down_) {
     brake_target_value_ = 2.5;
     error = (brake_target_value_ - cur_brake_v)*10;
     output = brake_pidcontrol_.ComputePID(error, 0.01, 0.5);
    control_command.set_brake(output);

     printf("[刹车]>>>目标值: %.3lf, 当前刹车电压值:%.3f\n", brake_target_value_, cur_brake_v);
     printf("[刹车]>>>差  值: %.3lf\n", error);
     printf("[刹车]>>>ouput: %.3lf\n", output);

     //if(fabs(error) < 0.5) {
       error = (thro_target_value_ - cur_thro_v)*10;
       output = throttle_pidcontrol_.ComputePID(error, 0.01, 0.5);
      control_command.set_throttle(output);
       printf("[油门]>>>目标值：%.3lf, 当前值:%.3f\n",thro_target_value_, cur_thro_v);
       printf("[油门]>>>差值: %.3lf\n", error);
       printf("[油门]>>>ouput:%.3lf\n", output);
      
    // } else {
    //   control_command.set_throttle(0);
    // }

  } else if(key_down_&&!key_up_){
    thro_target_value_ = 2.0;
    error = (thro_target_value_ - cur_thro_v)*10;
    output = throttle_pidcontrol_.ComputePID(error, 0.01, 0.5);
    printf("[油门]>>>目标值：%.3lf, 当前值:%.3f\n",thro_target_value_, cur_thro_v);
    printf("[油门]>>>差值: %.3lf\n", error);
    printf("[油门]>>>ouput:%.3lf\n", output);
    control_command.set_throttle(output);
   // if(fabs(error) < 0.5) {
      error = (brake_target_value_ - cur_brake_v)*10;
      output = brake_pidcontrol_.ComputePID(error, 0.01, 0.5);
      printf("[刹车]>>>目标值: %.3lf, 当前刹车电压值:%.3f\n", brake_target_value_, cur_brake_v);
      printf("[刹车]>>>差  值: %.3lf\n", error);
      printf("[刹车]>>>ouput: %.3lf\n", output);
      control_command.set_brake(output);
    // }else{
    //   control_command.set_brake(0);
    // }
  } else {
    thro_target_value_ = 2.0;
    error = (thro_target_value_ - cur_thro_v)*10;
    output = throttle_pidcontrol_.ComputePID(error, 0.01, 0.5);
    control_command.set_throttle(output);
    printf("[油门]>>>目标值：%.3lf, 当前值:%.3f\n",thro_target_value_, cur_thro_v);
    printf("[油门]>>>差值: %.3lf\n", error);
    printf("[油门]>>>ouput:%.3lf\n", output);

    brake_target_value_ = 2.5;
    error = (brake_target_value_ - cur_brake_v)*10;
    output = brake_pidcontrol_.ComputePID(error, 0.01, 0.5);
    control_command.set_brake(output);
    printf("[刹车]>>>目标值: %.3lf, 当前刹车电压值:%.3f\n", brake_target_value_, cur_brake_v);
    printf("[刹车]>>>差  值: %.3lf\n", error);
    printf("[刹车]>>>ouput: %.3lf\n", output);
  }
  printf("油门:Kp = %.3f; Ki = %.3f; Kd = %.3f\n", throttle_pidcontrol_.GetP(), throttle_pidcontrol_.GetI(), throttle_pidcontrol_.GetD());
  printf("刹车:Kp = %.3f; Ki = %.3f; Kd = %.3f\n", brake_pidcontrol_.GetP(), brake_pidcontrol_.GetI(), brake_pidcontrol_.GetD());
} 

void ControlComponent::OnPad(const std::shared_ptr<PadMessage> &pad) {
  std::lock_guard<std::mutex> lock(mutex_);
  pad_msg_.CopyFrom(*pad);
  ADEBUG << "Received Pad Msg:" << pad_msg_.DebugString();
  AERROR_IF(!pad_msg_.has_action()) << "pad message check failed!";
}

void ControlComponent::OnChassis(const std::shared_ptr<Chassis> &chassis) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_chassis_.CopyFrom(*chassis);
}

void ControlComponent::OnPlanning(
    const std::shared_ptr<ADCTrajectory> &trajectory) {
  ADEBUG << "Received chassis data: run trajectory callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_trajectory_.CopyFrom(*trajectory);
}

void ControlComponent::OnPlanningCommandStatus(
    const std::shared_ptr<external_command::CommandStatus>
        &planning_command_status) {
  ADEBUG << "Received plannning command status data: run planning command "
            "status callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  planning_command_status_.CopyFrom(*planning_command_status);
}

void ControlComponent::OnLocalization(
    const std::shared_ptr<LocalizationEstimate> &localization) {
  ADEBUG << "Received control data: run localization message callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_localization_.CopyFrom(*localization);
}

void ControlComponent::OnMonitor(
    const common::monitor::MonitorMessage &monitor_message) {
  for (const auto &item : monitor_message.item()) {
    if (item.log_level() == common::monitor::MonitorMessageItem::FATAL) {
      estop_ = true;
      return;
    }
  }
}

Status ControlComponent::ProduceControlCommand(
    ControlCommand *control_command) {
  Status status = CheckInput(&local_view_);
  // check data
  // if (!status.ok()) {
  //   AERROR_EVERY(100) << "Control input data failed: "
  //                     << status.error_message();
  //   control_command->mutable_engage_advice()->set_advice(
  //       apollo::common::EngageAdvice::DISALLOW_ENGAGE);
  //   control_command->mutable_engage_advice()->set_reason(
  //       status.error_message());
  //   estop_ = true;
  //   estop_reason_ = status.error_message();
  // } else {
  //   estop_ = false;
  //   Status status_ts = CheckTimestamp(local_view_);
  //   if (!status_ts.ok()) {
  //     AERROR << "Input messages timeout";
  //     // Clear trajectory data to make control stop if no data received again
  //     // next cycle.
  //     // keep the history trajectory for control compute.
  //     // latest_trajectory_.Clear();
  //     estop_ = true;
  //     status = status_ts;
  //     if (local_view_.chassis().driving_mode() !=
  //         apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
  //       control_command->mutable_engage_advice()->set_advice(
  //           apollo::common::EngageAdvice::DISALLOW_ENGAGE);
  //       control_command->mutable_engage_advice()->set_reason(
  //           status.error_message());
  //     }
  //   } else {
  //     control_command->mutable_engage_advice()->set_advice(
  //         apollo::common::EngageAdvice::READY_TO_ENGAGE);
  //     estop_ = false;
  //   }
  // }

  // check estop
  // estop_ = FLAGS_enable_persistent_estop
  //              ? estop_ || local_view_.trajectory().estop().is_estop()
  //              : local_view_.trajectory().estop().is_estop();

  // if (local_view_.trajectory().estop().is_estop()) {
  //   estop_ = true;
  //   estop_reason_ = "estop from planning : ";
  //   estop_reason_ += local_view_.trajectory().estop().reason();
  // }

  // if (local_view_.trajectory().trajectory_point().empty()) {
  //   AWARN_EVERY(100) << "planning has no trajectory point. ";
  //   estop_ = true;
  //   estop_reason_ = "estop for empty planning trajectory, planning headers: " +
  //                   local_view_.trajectory().header().ShortDebugString();
  // }

  // if (FLAGS_enable_gear_drive_negative_speed_protection) {
  //   const double kEpsilon = 0.001;
  //   auto first_trajectory_point = local_view_.trajectory().trajectory_point(0);
  //   if (local_view_.chassis().gear_location() == Chassis::GEAR_DRIVE &&
  //       first_trajectory_point.v() < -1 * kEpsilon) {
  //     estop_ = true;
  //     estop_reason_ = "estop for negative speed when gear_drive";
  //   }
  // }

  // if (!estop_) {
  //   if (local_view_.chassis().driving_mode() == Chassis::COMPLETE_MANUAL) {
  //     control_task_agent_.Reset();
  //     AINFO_EVERY(100) << "Reset Controllers in Manual Mode";
  //   }

  //   auto debug = control_command->mutable_debug()->mutable_input_debug();
  //   debug->mutable_localization_header()->CopyFrom(
  //       local_view_.localization().header());
  //   debug->mutable_canbus_header()->CopyFrom(local_view_.chassis().header());
  //   debug->mutable_trajectory_header()->CopyFrom(
  //       local_view_.trajectory().header());

  //   if (local_view_.trajectory().is_replan()) {
  //     latest_replan_trajectory_header_ = local_view_.trajectory().header();
  //   }

  //   if (latest_replan_trajectory_header_.has_sequence_num()) {
  //     debug->mutable_latest_replan_trajectory_header()->CopyFrom(
  //         latest_replan_trajectory_header_);
  //   }
  // }

  if (!local_view_.trajectory().trajectory_point().empty()) {
    // controller agent
    Status status_compute = control_task_agent_.ComputeControlCommand(
        &local_view_.localization(), &local_view_.chassis(),
        &local_view_.trajectory(), control_command);
    ADEBUG << "status_compute is " << status_compute;

    if (!status_compute.ok()) {
      AERROR << "Control main function failed"
             << " with localization: "
             << local_view_.localization().ShortDebugString()
             << " with chassis: " << local_view_.chassis().ShortDebugString()
             << " with trajectory: "
             << local_view_.trajectory().ShortDebugString()
             << " with cmd: " << control_command->ShortDebugString()
             << " status:" << status_compute.error_message();
      estop_ = true;
      estop_reason_ = status_compute.error_message();
      status = status_compute;
    }
  }

  // if planning set estop, then no control process triggered
  // if (estop_) {
  //   AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
  //   // set Estop command
  //   control_command->set_speed(0);
  //   control_command->set_throttle(0);
  //   control_command->set_brake(FLAGS_soft_estop_brake);
  //   control_command->set_gear_location(Chassis::GEAR_DRIVE);
  //   previous_steering_command_ =
  //       injector_->previous_control_command_mutable()->steering_target();
  //   control_command->set_steering_target(previous_steering_command_);
  // }
  // check signal
  // if (local_view_.trajectory().decision().has_vehicle_signal()) {
  //   control_command->mutable_signal()->CopyFrom(
  //       local_view_.trajectory().decision().vehicle_signal());
  // }
  return status;
}

bool ControlComponent::Proc() {
  const auto start_time = Clock::Now();

  chassis_reader_->Observe();
  const auto &chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg == nullptr) {
    AERROR << "Chassis msg is not ready!";
    // injector_->set_control_process(false);
    return false;
  }
  OnChassis(chassis_msg);

  double cur_thro_v = chassis_msg->throttle_percentage();
  double cur_brake_v = chassis_msg->brake_percentage();
  double cur_steer_angle = chassis_msg->steering_percentage();

  
  trajectory_reader_->Observe();
  const auto &trajectory_msg = trajectory_reader_->GetLatestObserved();
  if (trajectory_msg == nullptr) {
    AERROR << "planning msg is not ready!";
  } else {
    // Check if new planning data received.
    if (latest_trajectory_.header().sequence_num() !=
        trajectory_msg->header().sequence_num()) {
      OnPlanning(trajectory_msg);
    }
  }

  planning_command_status_reader_->Observe();
  const auto &planning_status_msg =
      planning_command_status_reader_->GetLatestObserved();
  if (planning_status_msg != nullptr) {
    OnPlanningCommandStatus(planning_status_msg);
    ADEBUG << "Planning command status msg is \n"
           << planning_command_status_.ShortDebugString();
  }
  // injector_->set_planning_command_status(planning_command_status_);

  localization_reader_->Observe();
  const auto &localization_msg = localization_reader_->GetLatestObserved();
  if (localization_msg == nullptr) {
    AERROR << "localization msg is not ready!";
    // injector_->set_control_process(false);
    return false;
  }
  OnLocalization(localization_msg);

  pad_msg_reader_->Observe();
  const auto &pad_msg = pad_msg_reader_->GetLatestObserved();
  if (pad_msg != nullptr) {
    OnPad(pad_msg);
  }

  {
    // TODO(SHU): to avoid redundent copy
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.mutable_chassis()->CopyFrom(latest_chassis_);
    local_view_.mutable_trajectory()->CopyFrom(latest_trajectory_);
    local_view_.mutable_localization()->CopyFrom(latest_localization_);
    if (pad_msg != nullptr) {
      local_view_.mutable_pad_msg()->CopyFrom(pad_msg_);
    }
  }

  // use control submodules
  // if (FLAGS_use_control_submodules) {
  //   local_view_.mutable_header()->set_lidar_timestamp(
  //       local_view_.trajectory().header().lidar_timestamp());
  //   local_view_.mutable_header()->set_camera_timestamp(
  //       local_view_.trajectory().header().camera_timestamp());
  //   local_view_.mutable_header()->set_radar_timestamp(
  //       local_view_.trajectory().header().radar_timestamp());
  //   common::util::FillHeader(FLAGS_control_local_view_topic, &local_view_);

  //   const auto end_time = Clock::Now();

  //   // measure latency
  //   static apollo::common::LatencyRecorder latency_recorder(
  //       FLAGS_control_local_view_topic);
  //   latency_recorder.AppendLatencyRecord(
  //       local_view_.trajectory().header().lidar_timestamp(), start_time,
  //       end_time);

  //   local_view_writer_->Write(local_view_);
  //   return true;
  // }

  // if (pad_msg != nullptr) {
  //   ADEBUG << "pad_msg: " << pad_msg_.ShortDebugString();
  //   if (pad_msg_.action() == DrivingAction::RESET) {
  //     AINFO << "Control received RESET action!";
  //     estop_ = false;
  //     estop_reason_.clear();
  //   }
  //   pad_received_ = true;
  // }

  // if (FLAGS_is_control_test_mode && FLAGS_control_test_duration > 0 &&
  //     (start_time - init_time_).ToSecond() > FLAGS_control_test_duration) {
  //   AERROR << "Control finished testing. exit";
  //   injector_->set_control_process(false);
  //   return false;
  // }

  // injector_->set_control_process(true);
  
  ControlCommand control_command;
  local_view_.mutable_chassis()->set_driving_mode(apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE);

  Status status;
  if (local_view_.chassis().driving_mode() ==

      apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    status = ProduceControlCommand(&control_command);
    ADEBUG << "Produce control command normal.";
  } else {
    ADEBUG << "Into reset control command.";
    ResetAndProduceZeroControlCommand(&control_command);
  }

  AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();

  if (pad_received_) {
    control_command.mutable_pad_msg()->CopyFrom(pad_msg_);
    pad_received_ = false;
  }

  // forward estop reason among following control frames.
  if (estop_) {
    control_command.mutable_header()->mutable_status()->set_msg(estop_reason_);
  }

  // set header
  control_command.mutable_header()->set_lidar_timestamp(
      local_view_.trajectory().header().lidar_timestamp());
  control_command.mutable_header()->set_camera_timestamp(
      local_view_.trajectory().header().camera_timestamp());
  control_command.mutable_header()->set_radar_timestamp(
      local_view_.trajectory().header().radar_timestamp());

  common::util::FillHeader(node_->Name(), &control_command);

  ADEBUG << control_command.ShortDebugString();
  if (FLAGS_is_control_test_mode) {
    ADEBUG << "Skip publish control command in test mode";
    return true;
  }

  if (fabs(control_command.debug().simple_lon_debug().vehicle_pitch()) <
      kDoubleEpsilon) {
    injector_->vehicle_state()->Update(local_view_.localization(),
                                       local_view_.chassis());
    GetVehiclePitchAngle(&control_command);
  }

  const auto end_time = Clock::Now();
  const double time_diff_ms = (end_time - start_time).ToSecond() * 1e3;
  ADEBUG << "total control time spend: " << time_diff_ms << " ms.";

  control_command.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  control_command.mutable_latency_stats()->set_total_time_exceeded(
      time_diff_ms > FLAGS_control_period * 1e3);
  ADEBUG << "control cycle time is: " << time_diff_ms << " ms.";
  status.Save(control_command.mutable_header()->mutable_status());

  // measure latency
  if (local_view_.trajectory().header().has_lidar_timestamp()) {
    static apollo::common::LatencyRecorder latency_recorder(
        FLAGS_control_command_topic);
    latency_recorder.AppendLatencyRecord(
        local_view_.trajectory().header().lidar_timestamp(), start_time,
        end_time);
  }

  // save current control command
  injector_->Set_pervious_control_command(&control_command);
  // injector_->previous_control_command_mutable()->CopyFrom(control_command);
  // injector_->previous_control_debug_mutable()->CopyFrom(
  //     injector_->control_debug_info());
  printf("[PID参数]>>>方向盘:Kp = %.3f; Ki = %.3f; Kd = %.3f\n", steer_pidcontrol_.GetP(), steer_pidcontrol_.GetI(), steer_pidcontrol_.GetD());
  printf("[PID参数]>>>刹  车:Kp = %.3lf; Ki = %.3lf; Kd = %.3lf\n", brake_pidcontrol_.GetP(), brake_pidcontrol_.GetI(), brake_pidcontrol_.GetD());
  printf("[PID参数]>>>油  门:Kp = %.3lf; Ki = %.3lf; Kd = %.3lf\n", throttle_pidcontrol_.GetP(), throttle_pidcontrol_.GetI(), throttle_pidcontrol_.GetD());


  double error = 0.0;
  double output = 0.0;
  if(!is_switch){
      steer_target_value_ = -control_command.steering_target()*scale;
      if (fabs(steer_target_value_) > 720) {
        steer_target_value_ = 720.0 * steer_target_value_ / fabs(steer_target_value_);
      }
      error =  steer_target_value_ -cur_steer_angle;
      output = steer_pidcontrol_.ComputePID(error, 0.01, 2.0);
      control_command.set_steering_rate(output);
    
      printf("[自动]>>>方向盘 误差：%.3lf; 输出: %.3lf\n", error, output);
      printf("[自动]>>>方向盘 目标角度: %.3lf, 当前角度：%.3lf\n", steer_target_value_, cur_steer_angle);
      printf("[自动]>>>方向盘 转向百分比：%.3lf\n", control_command.steering_target());
      printf("[自动]>>>方向盘 缩放系数: %.3lf.\n", scale);


      printf("[自动]>>>加速度 %.3lf\n", control_command.acceleration());
      thro_target_value_ = (control_command.acceleration() > 0)?0.76:1.8;
      brake_target_value_ = (control_command.acceleration() < 0)?0.8:2.5;
      error = (brake_target_value_ - cur_brake_v)*10;
      output = brake_pidcontrol_.ComputePID(error, 0.01, 0.5);
      control_command.set_brake(output);

      printf("[自动]>>>刹车 输出: %.3lf\n", output);
      printf("[自动]>>>刹车 目标电压: %.3lf, 当前电压：%.3lf\n", brake_target_value_, cur_brake_v);

      error = (thro_target_value_ - cur_thro_v)*10;
      output = throttle_pidcontrol_.ComputePID(error, 0.01, 0.5);
      control_command.set_throttle(output);

      printf("[自动]>>>油门 输出: %.3lf\n", output);
      printf("[自动]>>>油门 目标电压: %.3lf, 当前电压：%.3lf\n", thro_target_value_, cur_thro_v);
  } else {
    OnKeyBoard(cur_brake_v, cur_thro_v, cur_steer_angle, control_command);
  }

  control_cmd_writer_->Write(control_command);
  return true;
}

Status ControlComponent::CheckInput(LocalView *local_view) {
  ADEBUG << "Received localization:"
         << local_view->localization().ShortDebugString();
  ADEBUG << "Received chassis:" << local_view->chassis().ShortDebugString();

  if (!local_view->trajectory().estop().is_estop() &&
      local_view->trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    const std::string msg =
        absl::StrCat("planning has no trajectory point. planning_seq_num:",
                     local_view->trajectory().header().sequence_num());
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, msg);
  }

  for (auto &trajectory_point :
       *local_view->mutable_trajectory()->mutable_trajectory_point()) {
    if (std::abs(trajectory_point.v()) < FLAGS_minimum_speed_resolution &&
        std::abs(trajectory_point.a()) < FLAGS_max_acceleration_when_stopped) {
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(0.0);
    }
  }

  injector_->vehicle_state()->Update(local_view->localization(),
                                     local_view->chassis());

  return Status::OK();
}

Status ControlComponent::CheckTimestamp(const LocalView &local_view) {
  if (!FLAGS_enable_input_timestamp_check || FLAGS_is_control_test_mode) {
    ADEBUG << "Skip input timestamp check by gflags.";
    return Status::OK();
  }
  double current_timestamp = Clock::NowInSeconds();
  double localization_diff =
      current_timestamp - local_view.localization().header().timestamp_sec();
  if (localization_diff >
      (FLAGS_max_localization_miss_num * FLAGS_localization_period)) {
    AERROR << "Localization msg lost for " << std::setprecision(6)
           << localization_diff << "s";
    monitor_logger_buffer_.ERROR("Localization msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Localization msg timeout");
  }

  double chassis_diff =
      current_timestamp - local_view.chassis().header().timestamp_sec();
  if (chassis_diff > (FLAGS_max_chassis_miss_num * FLAGS_chassis_period)) {
    AERROR << "Chassis msg lost for " << std::setprecision(6) << chassis_diff
           << "s";
    monitor_logger_buffer_.ERROR("Chassis msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Chassis msg timeout");
  }

  double trajectory_diff =
      current_timestamp - local_view.trajectory().header().timestamp_sec();
  if (trajectory_diff >
      (FLAGS_max_planning_miss_num * FLAGS_trajectory_period)) {
    AERROR << "Trajectory msg lost for " << std::setprecision(6)
           << trajectory_diff << "s";
    monitor_logger_buffer_.ERROR("Trajectory msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Trajectory msg timeout");
  }
  return Status::OK();
}

void ControlComponent::ResetAndProduceZeroControlCommand(
    ControlCommand *control_command) {
  control_command->set_throttle(0.0);
  control_command->set_steering_target(0.0);
  control_command->set_steering_rate(0.0);
  control_command->set_speed(0.0);
  control_command->set_brake(0.0);
  control_command->set_gear_location(Chassis::GEAR_DRIVE);
  control_task_agent_.Reset();
  latest_trajectory_.mutable_trajectory_point()->Clear();
  latest_trajectory_.mutable_path_point()->Clear();
  trajectory_reader_->ClearData();
}

void ControlComponent::GetVehiclePitchAngle(ControlCommand *control_command) {
  double vehicle_pitch = injector_->vehicle_state()->pitch() * 180 / M_PI;
  control_command->mutable_debug()
      ->mutable_simple_lon_debug()
      ->set_vehicle_pitch(vehicle_pitch + FLAGS_pitch_offset_deg);
}

}  // namespace control
}  // namespace apollo