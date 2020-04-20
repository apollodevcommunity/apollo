/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/control/controller/new_controller.h"

#include <algorithm>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_util.h"
#include "modules/control/common/control_gflags.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace control {
    using apollo::common::ErrorCode;
    using apollo::common::Status;
    using apollo::common::TrajectoryPoint;
    using apollo::common::VehicleStateProvider;
    using apollo::common::time::Clock;

/*对构造函数进行初始化，在lon_controller.h文件125行中
声明了name_为字符串，将name_初始化为 Control_conf.proto
中的Lon_CONTROLLER控制器。*/
NewController::NewController()
    : name_(ControlConf_ControllerType_Name(ControlConf::NEW_CONTROLLER)) {
  if (FLAGS_enable_csv_debug) {
      /*根据google gflags中的介绍，所定义的gflags变量都可以通过FLAGS前缀加参数名访
    问，enable_csv_debug在control_gflags.cc中定义为DEFINE_bool(enable_csv_debug, 
    false, "True to write out csv debug file.")可知，如果为true，则写出CSV调试文件。*/

    /*以下是对参数是获取系统的时间，可参考：
    https://blog.csdn.net/u010087712/article/details/50731222/
    并将时间已一定的格式赋值给name_buffer,并创建空文件name_buff，如果创建失败，则错
    误提示并将FLAGS_enable_CSV_debug赋值false。创建成功，则在文件中打印以下信息 。
    这些信息在control_cmd.proto中均已定义。*/
    time_t rawtime;
    char name_buffer[80];
    std::time(&rawtime);
    std::tm time_tm;
    localtime_r(&rawtime, &time_tm);
    strftime(name_buffer, 80, "/tmp/speed_log__%F_%H%M%S.csv", &time_tm);
    speed_log_file_ = fopen(name_buffer, "w");
    if (speed_log_file_ == nullptr) {
    AERROR << "Fail to open file:" << name_buffer;
    FLAGS_enable_csv_debug = false;}
    if (speed_log_file_ != nullptr) {
      fprintf(speed_log_file_,
              "station_reference,"
              "station_error,"
              "preview_station_error,"
              "speed_reference,"
              "speed_error,"
              "preview_speed_reference,"
              "preview_speed_error,"
              "preview_acceleration_reference,"
              "acceleration_cmd_closeloop,"
              "acceleration_cmd,"
              "acceleration_lookup,"
              "speed_lookup,"
              "calibration_value,"
              "throttle_cmd,"
              "brake_cmd,"
              "is_full_stop,"
              "\r\n");
      fflush(speed_log_file_);
    }
    AINFO << name_ << " used.";
  }
}

// 如果不允许记录，则关闭记录文件并将文件指针指向空指针
void NewController::CloseLogFile() {
  if (FLAGS_enable_csv_debug) {
    if (speed_log_file_ != nullptr) {
      fclose(speed_log_file_);
      speed_log_file_ = nullptr;
    }
  }
}
/*以下为关闭记录文件，释放相关内存。*/
void NewController::Stop() { CloseLogFile(); }

NewController::~NewController() { CloseLogFile(); }

/*以下为对纵向控制器初始化函数，输入为控制算法参数配置，返回值为初始化的状态。*/
Status NewController::Init(const ControlConf *control_conf) {
  control_conf_ = control_conf;
  const NewControllerConf &new_controller_conf =control_conf_->new_controller_conf();

  /* 对位置PID和速度PID控制算法进行初始化，初始化的参数为control_conf.pb.txt
  文件中的参数，其中速度PID控制算法初始化为低速PID算法。以下为PID初始化函数。*/
  station_pid_controller_.Init(new_controller_conf.station_pid_conf());
  speed_pid_controller_.Init(new_controller_conf.low_speed_pid_conf());

  /*车辆参数的配置信息在modules/common/canbus/conf/canbus_conf.pb.txt中*/
  vehicle_param_.CopyFrom(common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param());

  /* 加载控制标定参数表，函数在后面*/
  LoadControlCalibrationTable(new_controller_conf);

  /*初始化完成，返回状态值*/
  controller_initialized_ = true;

  return Status::OK();
}

/*加载控制标定参数表，格式采用vector中的（x，y，z）方式。
参数表位于control_conf.pb.txt第168行至最后。如果存储失败，
则提示加载标定表失败*/
void NewController::LoadControlCalibrationTable(const NewControllerConf &new_controller_conf) {
  const auto &control_table = new_controller_conf.calibration_table();
  AINFO << "Control calibration table loaded";
  AINFO << "Control calibration table size is "
        << control_table.calibration_size();
  Interpolation2D::DataType xyz;
  for (const auto &calibration : control_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  control_interpolation_.reset(new Interpolation2D);
  CHECK(control_interpolation_->Init(xyz))
      << "Fail to load control calibration table";
}

/*以上是纵向控制算法的初始化，以下是本算法中最重要的部分，介绍了纵向控制
算法的实现。根据当前车辆的状态和目标轨迹计算制动及油门。
输入参数：车辆位置，车辆底盘状态（速度，加速度），规划模块生成的轨迹，控制命令；
返回：车辆计算的状态；*/
Status NewController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    control::ControlCommand *cmd) {
  /* （1）获取当前位置，车辆底盘信息，以及规划路径信息。*/
  localization_ = localization;
  chassis_ = chassis;
  trajectory_message_ = planning_published_trajectory;

  /*如果轨迹规划算法未设定完成，或轨迹规划算法序列和轨迹算法消息序列不相等，
  则重新设置指针指向轨迹规划算法。*/
  if (trajectory_analyzer_ == nullptr ||trajectory_analyzer_->seq_num() !=
          trajectory_message_->header().sequence_num()) {
    trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
  }

  const NewControllerConf &new_controller_conf =control_conf_->new_controller_conf();
  
  /*在control_cmd.proto中定义了集中控制算法的debug信息，例如纵向控制的debug，
  横向控制的debug，MPC控制的debug，输入的debug，下面依据是将本文件的
  debug信息指向纵向控制的debug信息。*/
  auto debug = cmd->mutable_debug()->mutable_simple_lon_debug();
  debug->Clear();

  double brake_cmd = 0.0;
  double throttle_cmd = 0.0;
  double ts = new_controller_conf.ts();
  /*预览时间定义为预览的视野与采样时间的乘积，在control_conf.pb.txt中可以看到预览视野为20。*/
  double preview_time = new_controller_conf.preview_window() * ts;

  /*以下为调用函数计算纵向的差值，函数实现在代码378行。
  输入：规划的轨迹，预览的时间，采样时间，debug信息。
  输出：相关偏差 */
  ComputeLongitudinalErrors(trajectory_analyzer_.get(), preview_time, ts,debug);
  
  /*判断车速，从而选择相应的不同车速下的PID控制参数。*/
  if (VehicleStateProvider::Instance()->linear_velocity() <=new_controller_conf.switch_speed()) {
    speed_pid_controller_.SetPID(new_controller_conf.low_speed_pid_conf());
  } else {
    speed_pid_controller_.SetPID(new_controller_conf.high_speed_pid_conf());
  }

  /*（5）根据位置误差进行PID运算，求出速度补偿值*/
  double speed_offset =station_pid_controller_.Control(debug->station_error(), ts);

  /*（6）计算速度输入，纵向车速=速度补偿+车速偏差。*/
  double speed_controller_input = 0.0;
  /****************************************************************************/
  /*****请老师在此处补全计算speed_controller_input代码*****/
  speed_controller_input = speed_offset + debug->speed_error();
  /****************************************************************************/


  /*（8）根据车速度补偿值，采用PID求取加速度补偿；*/
  double acceleration_cmd_closeloop = 0.0;
  acceleration_cmd_closeloop =speed_pid_controller_.Control(speed_controller_input, ts);

  /*（10）加速度=速度补偿+预览加速度+坡度补偿；*/
  /****************************************************************************/
  /*****请老师在此处补全计算acceleration_cmd代码*****/ 
  double acceleration_cmd =acceleration_cmd_closeloop + debug->preview_acceleration_reference() ;
  /****************************************************************************/


  // 关闭停车标志
  debug->set_is_full_stop(false);
  // 计算剩余规划轨迹
  GetPathRemain(debug);

  // At near-stop stage, replace the brake control command with the standstill
  // accleration if the former is even softer than the latter
  /*满足停车条件时，加速度=最大停止加速度；*/
  if ((trajectory_message_->trajectory_type() == apollo::planning::ADCTrajectory::NORMAL) &&
      ((std::fabs(debug->preview_acceleration_reference()) <=
            control_conf_->max_acceleration_when_stopped() &&
        std::fabs(debug->preview_speed_reference()) <=
            vehicle_param_.max_abs_speed_when_stopped()) ||
       std::abs(debug->path_remain()) <
           control_conf_->max_path_remain_when_stopped())) {
    acceleration_cmd =std::min(acceleration_cmd,new_controller_conf.standstill_acceleration());
    ADEBUG << "Stop location reached";
    debug->set_is_full_stop(true);
  }

  /*油门下边界=油门死区和油门最小值的较大一个；刹车下边界=刹车死区和刹车最小值的较大一个；*/
  double throttle_lowerbound =std::max(vehicle_param_.throttle_deadzone(),new_controller_conf.throttle_minimum_action());
  double brake_lowerbound =std::max(vehicle_param_.brake_deadzone(),new_controller_conf.brake_minimum_action());

  double calibration_value = 0.0;
  double acceleration_lookup =acceleration_cmd;

  /*（11）获取刹车/油门命令
如果用标定表中的预览速度，标定值为预览参考速度和查表加速度的简单线性插值，
否则为车标表和查表加速的简单线性插值；
*/
  calibration_value = control_interpolation_->Interpolate(std::make_pair(chassis_->speed_mps(), acceleration_lookup));
  
  /*如果加速度命令大于0，则判断查表得到的油门/刹车命令值，
如果得到的值大于0，则油门取标定值和油门最小值中较大一个，刹车命令为0；
否则油门取下边界，刹车命令为0；
如果加速度命令不大于0，则油门命令为0，刹车命令为较大一个。
*/
  if (acceleration_lookup >= 0) {
    if (calibration_value >= 0) {
      throttle_cmd = std::max(calibration_value, throttle_lowerbound);
    } else {
      throttle_cmd = throttle_lowerbound;
    }
    brake_cmd = 0.0;
  } else {
    throttle_cmd = 0.0;
    if (calibration_value >= 0) {
      brake_cmd = brake_lowerbound;
    } else {
      brake_cmd = std::max(-calibration_value, brake_lowerbound);
    }
  }

  debug->set_speed_offset(speed_offset);
  debug->set_acceleration_cmd(acceleration_cmd);
  debug->set_throttle_cmd(throttle_cmd);
  debug->set_brake_cmd(brake_cmd);
  debug->set_acceleration_lookup(acceleration_lookup);
  debug->set_speed_lookup(chassis_->speed_mps());
  debug->set_calibration_value(calibration_value);
  debug->set_acceleration_cmd_closeloop(acceleration_cmd_closeloop);

  if (FLAGS_enable_csv_debug && speed_log_file_ != nullptr) {
    fprintf(speed_log_file_,
            "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f,"
            "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %d,\r\n",
            debug->station_reference(), debug->station_error(),
            debug->preview_station_error(),debug->speed_reference(), debug->speed_error(),
            debug->preview_speed_reference(),debug->preview_speed_error(),
            debug->preview_acceleration_reference(), acceleration_cmd_closeloop,
            acceleration_cmd, debug->acceleration_lookup(),
            debug->speed_lookup(), calibration_value, throttle_cmd, brake_cmd,
            debug->is_full_stop());
  }

  /*如果车加速度，则不用管油门和刹车；*/
  // if the car is driven by acceleration, disgard the cmd->throttle and brake

  /****************************************************************************/
  /*****请老师在此处补全计算cmd控制命令代码*****/
  cmd->set_throttle(throttle_cmd);
  cmd->set_brake(brake_cmd);
  //对于有些车辆只支持加速度接口控制，也可将最终加速度补偿值直接下发至canbus，作为控制命令
  //cmd->set_acceleration(acceleration_cmd);
  /****************************************************************************/


  /*如果目前车速为停止车速，或者规划档位和底盘档位均为空挡，则设置档位为空挡，否则
档位为底盘档位；
*/
  if (std::fabs(VehicleStateProvider::Instance()->linear_velocity()) <=
          vehicle_param_.max_abs_speed_when_stopped() ||
      chassis->gear_location() == trajectory_message_->gear() ||
      chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
    cmd->set_gear_location(trajectory_message_->gear());
  } else {
    cmd->set_gear_location(chassis->gear_location());
  }

  return Status::OK();
}

Status NewController::Reset() {
  speed_pid_controller_.Reset();
  station_pid_controller_.Reset();
  return Status::OK();
}

std::string NewController::Name() const { return name_; }

/*以下计算纵向的差值*/
void NewController::ComputeLongitudinalErrors(
    const TrajectoryAnalyzer *trajectory_analyzer, const double preview_time,
    const double ts, SimpleLongitudinalDebug *debug) {
  // the decomposed vehicle motion onto Frenet frame 分解的车辆运动到Frenet坐标系
  // s: longitudinal accumulated distance along reference trajectory 沿参考轨迹的纵向累积距离
  // s_dot: longitudinal velocity along reference trajectory 沿参考轨迹的纵向速度
  // d: lateral distance w.r.t. reference trajectory 横向距离w.r.t. 参考轨迹
  // d_dot: lateral distance change rate, i.e. dd/dt  横向距离变化率

  /*含义分别为纵向车与参考轨迹的距离，沿纵向参考轨迹的车速，横向与参考
  轨迹的距离，横向距离的变化率 。*/
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;
  
  /*（2）按距离最小查找与当前车辆位置最匹配的参考点。
该点就是车辆目前的位置。
首先路径规划模块应该给出所有的规划点（规划点是以矩阵形式存储的，我理解此时
的规划点是车辆下一时刻可能到达的所有可能点，具体是啥应该看规划的代码），
然后下列函数就是查找所有给出的规划点中距离车辆目前位置最近的点。*/
  auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
      VehicleStateProvider::Instance()->x(),
      VehicleStateProvider::Instance()->y());

  /*（3）计算纵向误差的相关参数；
车辆目前位置到匹配点的纵向距离偏差，横向距离偏差，沿匹配点的横纵向车速；
具体如何实现可参考如下文章：
https://blog.csdn.net/u013914471/article/details/83748571
*/
  trajectory_analyzer->ToTrajectoryFrame(
      VehicleStateProvider::Instance()->x(),
      VehicleStateProvider::Instance()->y(),
      VehicleStateProvider::Instance()->heading(),
      VehicleStateProvider::Instance()->linear_velocity(), matched_point,
      &s_matched, &s_dot_matched, &d_matched, &d_dot_matched);

/*通过common/planning/proto/planning.proto可以知道，
TrajectoryPoint=path data+speed data
以下是找到相应的规划点，该点在时间上与目前车辆的时间最接近，即规划的
下一个时刻的点的位置。*/
  double current_control_time = Clock::NowInSeconds();
  double preview_control_time = current_control_time + preview_time;

  TrajectoryPoint reference_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          current_control_time);
  TrajectoryPoint preview_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          preview_control_time);
  debug->set_current_station(s_matched);
  /****************************************************************************/
  /*****请老师在此处补全station_error和speed_error代码*****/
  debug->set_station_error(reference_point.path_point().s() - s_matched);
  debug->set_speed_error(reference_point.v() - s_dot_matched);
  /****************************************************************************/

  debug->set_preview_acceleration_reference(preview_point.a());
}

/*以下函数为计算剩余规划点的个数，当某个点车速过小且加速度过小时，判断在该点车辆停止。*/
// TODO(all): Refactor and simplify
void NewController::GetPathRemain(SimpleLongitudinalDebug *debug) {
  int stop_index = 0;
  if (trajectory_message_->gear() == canbus::Chassis::GEAR_DRIVE) {
    while (stop_index < trajectory_message_->trajectory_point_size()) {
      if (fabs(trajectory_message_->trajectory_point(stop_index).v()) < 1e-3 &&
          trajectory_message_->trajectory_point(stop_index).a() > -0.01 &&
          trajectory_message_->trajectory_point(stop_index).a() < 0.0) {
        break;
      } else {
        ++stop_index;
      }
    }
  } 

  if (stop_index == trajectory_message_->trajectory_point_size()) {
    --stop_index;
    if (fabs(trajectory_message_->trajectory_point(stop_index).v()) < 0.1) {
      ADEBUG << "the last point is selected as parking point";
    } else {
      ADEBUG << "the last point found in path and speed > speed_deadzone";
      debug->set_path_remain(10000);
    }
  }
  debug->set_path_remain(
      trajectory_message_->trajectory_point(stop_index).path_point().s() -
      debug->current_station());
}

}  // namespace control
}  // namespace apollo