本文档将介绍如何使用 Apollo 的工具，生成适配 Apollo 的 CANBUS 代码。

## 前提条件

您需要先阅读以下内容:

- [概览](../应用实践/车辆集成教程/准备符合Apollo协议的车辆/概览.md)

- [符合 Apollo 线控标准的车辆](../应用实践/车辆集成教程/准备符合Apollo协议的车辆/符合Apollo线控标准的车辆.md)

## 将 CANBUS 适配 Apollo

### 1. DBC 文件转换成 canbus 模板代码

Canbus 适配代码可以使用 Apollo 的工具生成。在转换代码前，要保证 DBC 按照上述 DBC 文件要求完成，并通过 gedit 打开 DBC 文件，另存转码为 UTF-8 格式保存。

1. 将 DBC 文件放置指定目录 `apollo/modules/tools/gen_vehicle_protocol` 内。

2. 修改 DBC 转换脚本的配置文件。

   下面以 GE3 车型添加为例，在 `apollo/modules/tools/gen_vehicle_protocol` 目录下，复制默认存在的 `mkz_conf.yml` 文件并重命名为 `ge3_conf.yml`，修改该配置文件，如下图所示：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_7fbaf21.png)

- `dbc_file`：填写对应你的 DBC 文件名称，DBC 文件名称一般以车型名称命名，并以 `.dbc` 结束。

- `protocol_conf`：与上述 DBC 文件名称命名相同，填写`ge3.yml`。

- `car_type`：填入车型名称。

- `sender_list：[ ] `：发送列表，这里默认为空。

- `sender`：此处修改为与 DBC 内定义的 Apollo 的名称一致，ge3 的 DBC 内定义 Apollo 名称为 SCU。

3. 完成 `ge3_conf.yml` 配置文件设置，启动 docker，进入 Apollo 的容器后，在 `apollo/modules/tools/gen_vehicle_protocol` 目录下，找到 DBC 转化工具 `gen.py`，执行代码：

   ```
   cd modules/tools/gen_vehicle_protocol
   python gen.py ge3_conf.ymal
   ```

   执行完成上述脚本后，在终端内会显示生成 5 个控制协议，11 个反馈协议。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_f1f6517.png)

   这时在 `apollo/modules/tools/gen_vehicle_protocol` 目录下，会生成一个 `output` 文件夹，文件夹内有 2 个文件夹：`proto` 和 `vehicle` 文件夹。这两个文件内的代码内容就是要适配 canbus 的基本代码模板。把文件内的代码 **拷贝** 到 Apollo 的 canbus 层内，进行代码适配添加。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_e0260ae.png)

> 注意：把这个 output 文件夹内生成的代码模板拷贝至相应的 Apollo 目录后，要删除该文件夹，如果不删除该文件夹，后期编译 Apollo 时会报错。该文件夹有保护权限，请在 Apollo 的 docker 内执行删除代码。

```
rm -rf output/
```

### 2. 适配代码合入 Apollo 文件内

下面以添加 ge3 车型为例，将该代码添加至 Apollo 内：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_7f49ca5.png)

1. 将 `apollo/modules/tools/gen_vehicle_protocol/output/proto` 文件夹内 `ge3.proto` 文件拷贝至 `apollo/modules/canbus/proto` 文件夹内，并在该文件夹内修改 `chassis_detail.proto`，在该文件头部添加头文件 `import "modules/canbus/proto/ge3.proto"`。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_7d03448.png)

在 `message ChassisDetail{}` 结构体内的最后一行添加要增加的新车型变量定义：`Ge3 ge3 = 21`：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_ce56d91.png)

在 `pollo/modules/canbus/proto`目录的 `BUILD` 文件内添加上述新 `proto` 的依赖：`"ge3.proto"`：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_ccdf17d.png)

2. 将 `apollo/modules/tools/gen_vehicle_protocol/output/vehicle/` 内的 ge3 文件夹拷贝至 `apollo/modules/canbus/vehicle/` 文件夹下。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_20984df.png)

### 3. 实现新的车辆控制逻辑

实现新的车辆控制逻辑，在 `apollo/modules/canbus/vehicle/ge3/ge3_controller.cc` 文件编写控制逻辑代码。

控制代码主要包含将解析的底盘反馈报文的信息，通过 `chassis` 和 `chassis_detail` 广播出车辆底盘信息。

`chassis` 主要包括获取底盘的车速、轮速、发动机转速、踏板反馈、转角反馈等信息， `chassis_detail` 是每一帧报文的实际信息，这一部分编写的代码如下所示：

```
  // 3
  chassis_.set_engine_started(true);

  // check if there is not ge3, no chassis detail can be retrieved and return
  if (!chassis_detail.has_ge3()) {
    AERROR << "NO GE3 chassis information!";
    return chassis_;
  }
  Ge3 ge3 = chassis_detail.ge3();
  // 5
  if (ge3.has_scu_bcs_3_308()) {
    Scu_bcs_3_308 scu_bcs_3_308 = ge3.scu_bcs_3_308();
    if (scu_bcs_3_308.has_bcs_rrwheelspd()) {
      if (chassis_.has_wheel_speed()) {
        chassis_.mutable_wheel_speed()->set_is_wheel_spd_rr_valid(
            scu_bcs_3_308.bcs_rrwheelspdvd());
        chassis_.mutable_wheel_speed()->set_wheel_direction_rr(
            (WheelSpeed::WheelSpeedType)scu_bcs_3_308.bcs_rrwheeldirection());
        chassis_.mutable_wheel_speed()->set_wheel_spd_rr(
            scu_bcs_3_308.bcs_rrwheelspd());
      }
    }

    if (scu_bcs_3_308.has_bcs_rlwheelspd()) {
      if (chassis_.has_wheel_speed()) {
        chassis_.mutable_wheel_speed()->set_is_wheel_spd_rl_valid(
            scu_bcs_3_308.bcs_rlwheelspdvd());
        chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
            (WheelSpeed::WheelSpeedType)scu_bcs_3_308.bcs_rlwheeldirection());
        chassis_.mutable_wheel_speed()->set_wheel_spd_rl(
            scu_bcs_3_308.bcs_rlwheelspd());
      }
    }

    if (scu_bcs_3_308.has_bcs_frwheelspd()) {
      if (chassis_.has_wheel_speed()) {
        chassis_.mutable_wheel_speed()->set_is_wheel_spd_fr_valid(
            scu_bcs_3_308.bcs_frwheelspdvd());
        chassis_.mutable_wheel_speed()->set_wheel_direction_fr(
            (WheelSpeed::WheelSpeedType)scu_bcs_3_308.bcs_frwheeldirection());
        chassis_.mutable_wheel_speed()->set_wheel_spd_fr(
            scu_bcs_3_308.bcs_frwheelspd());
      }
    }

    if (scu_bcs_3_308.has_bcs_flwheelspd()) {
      if (chassis_.has_wheel_speed()) {
        chassis_.mutable_wheel_speed()->set_is_wheel_spd_fl_valid(
            scu_bcs_3_308.bcs_flwheelspdvd());
        chassis_.mutable_wheel_speed()->set_wheel_direction_fl(
            (WheelSpeed::WheelSpeedType)scu_bcs_3_308.bcs_flwheeldirection());
        chassis_.mutable_wheel_speed()->set_wheel_spd_fl(
            scu_bcs_3_308.bcs_flwheelspd());
      }
    }
  }

  if (ge3.has_scu_bcs_2_307() && ge3.scu_bcs_2_307().has_bcs_vehspd()) {
    chassis_.set_speed_mps(
        static_cast<float>(ge3.scu_bcs_2_307().bcs_vehspd()));
  } else {
    chassis_.set_speed_mps(0);
  }

  // 7
  // ge3 only has fuel percentage
  // to avoid confusing, just don't set
  chassis_.set_fuel_range_m(0);

  if (ge3.has_scu_vcu_1_312() && ge3.scu_vcu_1_312().has_vcu_accpedact()) {
    chassis_.set_throttle_percentage(
        static_cast<float>(ge3.scu_vcu_1_312().vcu_accpedact()));
  } else {
    chassis_.set_throttle_percentage(0);
  }
  // 9
  if (ge3.has_scu_bcs_1_306() && ge3.scu_bcs_1_306().has_bcs_brkpedact()) {
    chassis_.set_brake_percentage(
        static_cast<float>(ge3.scu_bcs_1_306().bcs_brkpedact()));
  } else {
    chassis_.set_brake_percentage(0);
  }
  // 23, previously 10
  if (ge3.has_scu_vcu_1_312() && ge3.scu_vcu_1_312().has_vcu_gearact()) {
    switch (ge3.scu_vcu_1_312().vcu_gearact()) {
      case Scu_vcu_1_312::VCU_GEARACT_INVALID: {
        chassis_.set_gear_location(Chassis::GEAR_INVALID);
      } break;
      case Scu_vcu_1_312::VCU_GEARACT_DRIVE: {
        chassis_.set_gear_location(Chassis::GEAR_DRIVE);
      } break;
      case Scu_vcu_1_312::VCU_GEARACT_NEUTRAL: {
        chassis_.set_gear_location(Chassis::GEAR_NEUTRAL);
      } break;
      case Scu_vcu_1_312::VCU_GEARACT_REVERSE: {
        chassis_.set_gear_location(Chassis::GEAR_REVERSE);
      } break;
      case Scu_vcu_1_312::VCU_GEARACT_PARK: {
        chassis_.set_gear_location(Chassis::GEAR_PARKING);
      } break;
      default:
        chassis_.set_gear_location(Chassis::GEAR_INVALID);
        break;
    }
  } else {
    chassis_.set_gear_location(Chassis::GEAR_INVALID);
  }

  // 11
  if (ge3.has_scu_eps_311() && ge3.scu_eps_311().has_eps_steerangle()) {
    chassis_.set_steering_percentage(
        static_cast<float>(ge3.scu_eps_311().eps_steerangle() /
                           vehicle_params_.max_steer_angle() * M_PI / 1.80));
  } else {
    chassis_.set_steering_percentage(0);
  }

  // 13
  if (ge3.has_scu_epb_310() && ge3.scu_epb_310().has_epb_sysst()) {
    chassis_.set_parking_brake(ge3.scu_epb_310().epb_sysst() ==
                               Scu_epb_310::EPB_SYSST_APPLIED);
  } else {
    chassis_.set_parking_brake(false);
  }

  // 14, 15: ge3 light control
  if (ge3.has_scu_bcm_304() && ge3.scu_bcm_304().has_bcm_highbeamst() &&
      Scu_bcm_304::BCM_HIGHBEAMST_ACTIVE ==
          ge3.scu_bcm_304().bcm_highbeamst()) {
    if (chassis_.has_signal()) {
      chassis_.mutable_signal()->set_high_beam(true);
    }
  } else {
    if (chassis_.has_signal()) {
      chassis_.mutable_signal()->set_high_beam(false);
    }
  }

  // 16, 17
  if (ge3.has_scu_bcm_304()) {
    Scu_bcm_304 scu_bcm_304 = ge3.scu_bcm_304();
    if (scu_bcm_304.has_bcm_leftturnlampst() &&
        Scu_bcm_304::BCM_LEFTTURNLAMPST_ACTIVE ==
            scu_bcm_304.bcm_leftturnlampst()) {
      chassis_.mutable_signal()->set_turn_signal(
          common::VehicleSignal::TURN_LEFT);
    } else if (scu_bcm_304.has_bcm_rightturnlampst() &&
               Scu_bcm_304::BCM_RIGHTTURNLAMPST_ACTIVE ==
                   scu_bcm_304.bcm_rightturnlampst()) {
      chassis_.mutable_signal()->set_turn_signal(
          common::VehicleSignal::TURN_RIGHT);
    } else {
      chassis_.mutable_signal()->set_turn_signal(
          common::VehicleSignal::TURN_NONE);
    }
  } else {
    chassis_.mutable_signal()->set_turn_signal(
        common::VehicleSignal::TURN_NONE);
  }
  // 18
  if (ge3.has_scu_bcm_304() && ge3.scu_bcm_304().has_bcm_hornst() &&
      Scu_bcm_304::BCM_HORNST_ACTIVE == ge3.scu_bcm_304().bcm_hornst()) {
    chassis_.mutable_signal()->set_horn(true);
  } else {
    chassis_.mutable_signal()->set_horn(false);
  }

  // vin number will be written into KVDB once.
  chassis_.mutable_vehicle_id()->set_vin("");
  if (ge3.has_scu_1_301() && ge3.has_scu_2_302() && ge3.has_scu_3_303()) {
    Scu_1_301 scu_1_301 = ge3.scu_1_301();
    Scu_2_302 scu_2_302 = ge3.scu_2_302();
    Scu_3_303 scu_3_303 = ge3.scu_3_303();
    if (scu_2_302.has_vin00() && scu_2_302.has_vin01() &&
        scu_2_302.has_vin02() && scu_2_302.has_vin03() &&
        scu_2_302.has_vin04() && scu_2_302.has_vin05() &&
        scu_2_302.has_vin06() && scu_2_302.has_vin07() &&
        scu_3_303.has_vin08() && scu_3_303.has_vin09() &&
        scu_3_303.has_vin10() && scu_3_303.has_vin11() &&
        scu_3_303.has_vin12() && scu_3_303.has_vin13() &&
        scu_3_303.has_vin14() && scu_3_303.has_vin15() &&
        scu_1_301.has_vin16()) {
      int n[17];
      n[0] = scu_2_302.vin00();
      n[1] = scu_2_302.vin01();
      n[2] = scu_2_302.vin02();
      n[3] = scu_2_302.vin03();
      n[4] = scu_2_302.vin04();
      n[5] = scu_2_302.vin05();
      n[6] = scu_2_302.vin06();
      n[7] = scu_2_302.vin07();
      n[8] = scu_3_303.vin08();
      n[9] = scu_3_303.vin09();
      n[10] = scu_3_303.vin10();
      n[11] = scu_3_303.vin11();
      n[12] = scu_3_303.vin12();
      n[13] = scu_3_303.vin13();
      n[14] = scu_3_303.vin14();
      n[15] = scu_3_303.vin15();
      n[16] = scu_1_301.vin16();

      char ch[17];
      memset(&ch, '\0', sizeof(ch));
      for (int i = 0; i < 17; i++) {
        ch[i] = static_cast<char>(n[i]);
      }
      if (chassis_.has_vehicle_id()) {
        chassis_.mutable_vehicle_id()->set_vin(ch);
      }
    }
  }

  // give engage_advice based on error_code and canbus feedback
  if (chassis_error_mask_) {
    if (chassis_.has_engage_advice()) {
      chassis_.mutable_engage_advice()->set_advice(
          apollo::common::EngageAdvice::DISALLOW_ENGAGE);
      chassis_.mutable_engage_advice()->set_reason("Chassis error!");
    }
  } else if (chassis_.parking_brake() || CheckSafetyError(chassis_detail)) {
    if (chassis_.has_engage_advice()) {
      chassis_.mutable_engage_advice()->set_advice(
          apollo::common::EngageAdvice::DISALLOW_ENGAGE);
      chassis_.mutable_engage_advice()->set_reason(
          "Vehicle is not in a safe state to engage!");
    }
  } else {
    if (chassis_.has_engage_advice()) {
      chassis_.mutable_engage_advice()->set_advice(
          apollo::common::EngageAdvice::READY_TO_ENGAGE);
    }
  }
```

设置自动驾驶模式，编辑相关使能逻辑，在 Apollo 中，车辆的驾驶模式主要包含：

- 完全自动驾驶模式（`COMPLETE_AUTO_DRIVE`）：横向、纵向都使能，

- 横向自动驾驶模式（`AUTO_STEER_ONLY`）：横向使能，纵向不使能，

- 纵向自动驾驶模式（`AUTO_SPEED_ONLY`）：横向不使能，纵向使能。

车辆使能控制信号控制逻辑如下所示：

```
ErrorCode Ge3Controller::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  pc_bcs_202_->set_pc_brkpedenable(Pc_bcs_202::PC_BRKPEDENABLE_ENABLE);
  pc_vcu_205_->set_pc_accpedenable(Pc_vcu_205::PC_ACCPEDENABLE_ENABLE);
  pc_vcu_205_->set_pc_gearenable(Pc_vcu_205::PC_GEARENABLE_ENABLE);
  pc_epb_203_->set_pc_epbenable(Pc_epb_203::PC_EPBENABLE_ENABLE);
  pc_eps_204_->set_pc_steerenable(Pc_eps_204::PC_STEERENABLE_ENABLE);

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  // If the auto mode can be set normally, the harzad lamp should be off.
  pc_bcm_201_->set_pc_hazardlampreq(Pc_bcm_201::PC_HAZARDLAMPREQ_NOREQ);
  AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
  return ErrorCode::OK;
}

ErrorCode Ge3Controller::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL OK.";
  return ErrorCode::OK;
}

ErrorCode Ge3Controller::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode";
    return ErrorCode::OK;
  }
  pc_bcs_202_->set_pc_brkpedenable(Pc_bcs_202::PC_BRKPEDENABLE_DISABLE);
  pc_vcu_205_->set_pc_accpedenable(Pc_vcu_205::PC_ACCPEDENABLE_DISABLE);
  pc_vcu_205_->set_pc_gearenable(Pc_vcu_205::PC_GEARENABLE_DISABLE);
  pc_epb_203_->set_pc_epbenable(Pc_epb_203::PC_EPBENABLE_DISABLE);
  pc_eps_204_->set_pc_steerenable(Pc_eps_204::PC_STEERENABLE_ENABLE);

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_STEER_ONLY);
  // If the auto mode can be set normally, the harzad lamp should be off.
  pc_bcm_201_->set_pc_hazardlampreq(Pc_bcm_201::PC_HAZARDLAMPREQ_NOREQ);
  AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
  return ErrorCode::OK;
}

ErrorCode Ge3Controller::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  pc_bcs_202_->set_pc_brkpedenable(Pc_bcs_202::PC_BRKPEDENABLE_ENABLE);
  pc_vcu_205_->set_pc_accpedenable(Pc_vcu_205::PC_ACCPEDENABLE_ENABLE);
  pc_vcu_205_->set_pc_gearenable(Pc_vcu_205::PC_GEARENABLE_ENABLE);
  pc_epb_203_->set_pc_epbenable(Pc_epb_203::PC_EPBENABLE_ENABLE);
  pc_eps_204_->set_pc_steerenable(Pc_eps_204::PC_STEERENABLE_DISABLE);

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_SPEED_ONLY);
  // If the auto mode can be set normally, the harzad lamp should be off.
  pc_bcm_201_->set_pc_hazardlampreq(Pc_bcm_201::PC_HAZARDLAMPREQ_NOREQ);
  AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
  return ErrorCode::OK;
}
```

> 说明：添加控制信号的相关功能，必须要添加的控制信号包括车辆的油门、刹车踏板控制，转向控制，和档位控制。

其它控制信号包括车大灯控制、喇叭控制、转向灯控制、电子手刹控制。代码如下所示：

```
// NEUTRAL, REVERSE, DRIVE
void Ge3Controller::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }
  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      pc_vcu_205_->set_pc_gearreq(Pc_vcu_205::PC_GEARREQ_NEUTRAL);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      pc_vcu_205_->set_pc_gearreq(Pc_vcu_205::PC_GEARREQ_REVERSE);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      pc_vcu_205_->set_pc_gearreq(Pc_vcu_205::PC_GEARREQ_DRIVE);
      break;
    }
    case Chassis::GEAR_PARKING: {
      pc_vcu_205_->set_pc_gearreq(Pc_vcu_205::PC_GEARREQ_PARK);
      break;
    }
    case Chassis::GEAR_LOW: {
      pc_vcu_205_->set_pc_gearreq(Pc_vcu_205::PC_GEARREQ_INVALID);
      break;
    }
    case Chassis::GEAR_NONE: {
      pc_vcu_205_->set_pc_gearreq(Pc_vcu_205::PC_GEARREQ_INVALID);
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      pc_vcu_205_->set_pc_gearreq(Pc_vcu_205::PC_GEARREQ_INVALID);
      break;
    }
    default: {
      pc_vcu_205_->set_pc_gearreq(Pc_vcu_205::PC_GEARREQ_INVALID);
      break;
    }
  }
}

// brake with new acceleration
// acceleration:0.00~99.99, unit:
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
// -> pedal
void Ge3Controller::Brake(double pedal) {
  // Update brake value based on mode
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  pc_bcs_202_->set_pc_brkpedreq(pedal);
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void Ge3Controller::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  pc_vcu_205_->set_pc_accpedreq(pedal);
}

// ge3 default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void Ge3Controller::Steer(double angle) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  pc_eps_204_->set_pc_steerangreq(real_angle)->set_pc_steerspdreq(500);
}

// drive with acceleration/deceleration
// acc:-7.0 ~ 5.0, unit:m/s^2
void Ge3Controller::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  // None
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void Ge3Controller::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  const double real_angle_spd =
      ProtocolData<::apollo::canbus::ChassisDetail>::BoundedValue(
          vehicle_params_.min_steer_angle_rate() / M_PI * 180,
          vehicle_params_.max_steer_angle_rate() / M_PI * 180,
          vehicle_params_.max_steer_angle_rate() / M_PI * 180 * angle_spd /
              100.0);
  pc_eps_204_->set_pc_steerangreq(real_angle)
      ->set_pc_steerspdreq(static_cast<int>(real_angle_spd));
}

void Ge3Controller::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    pc_epb_203_->set_pc_epbreq(Pc_epb_203::PC_EPBREQ_APPLY);
  } else {
    pc_epb_203_->set_pc_epbreq(Pc_epb_203::PC_EPBREQ_RELEASE);
  }
}

void Ge3Controller::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    pc_bcm_201_->set_pc_lowbeamreq(Pc_bcm_201::PC_LOWBEAMREQ_NOREQ);
    pc_bcm_201_->set_pc_highbeamreq(Pc_bcm_201::PC_HIGHBEAMREQ_REQ);
  } else if (command.signal().low_beam()) {
    pc_bcm_201_->set_pc_lowbeamreq(Pc_bcm_201::PC_LOWBEAMREQ_REQ);
    pc_bcm_201_->set_pc_highbeamreq(Pc_bcm_201::PC_HIGHBEAMREQ_NOREQ);
  } else {
    pc_bcm_201_->set_pc_lowbeamreq(Pc_bcm_201::PC_LOWBEAMREQ_NOREQ);
    pc_bcm_201_->set_pc_highbeamreq(Pc_bcm_201::PC_HIGHBEAMREQ_NOREQ);
  }
}

void Ge3Controller::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    pc_bcm_201_->set_pc_hornreq(Pc_bcm_201::PC_HORNREQ_REQ);
  } else {
    pc_bcm_201_->set_pc_hornreq(Pc_bcm_201::PC_HORNREQ_NOREQ);
  }
}

void Ge3Controller::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  auto signal = command.signal().turn_signal();
  if (signal == common::VehicleSignal::TURN_LEFT) {
    pc_bcm_201_->set_pc_leftturnlampreq(Pc_bcm_201::PC_LEFTTURNLAMPREQ_REQ);
    pc_bcm_201_->set_pc_rightturnlampreq(Pc_bcm_201::PC_RIGHTTURNLAMPREQ_NOREQ);
  } else if (signal == common::VehicleSignal::TURN_RIGHT) {
    pc_bcm_201_->set_pc_leftturnlampreq(Pc_bcm_201::PC_LEFTTURNLAMPREQ_NOREQ);
    pc_bcm_201_->set_pc_rightturnlampreq(Pc_bcm_201::PC_RIGHTTURNLAMPREQ_REQ);
  } else {
    pc_bcm_201_->set_pc_leftturnlampreq(Pc_bcm_201::PC_LEFTTURNLAMPREQ_NOREQ);
    pc_bcm_201_->set_pc_rightturnlampreq(Pc_bcm_201::PC_RIGHTTURNLAMPREQ_NOREQ);
  }
}
```

添加 `CheckResponse` 逻辑，Apollo 程序内增加了对车辆底层是否在自动驾驶模式的监控，即车辆横向、驱动、制动模块的驾驶模式反馈是否处于自动驾驶状态。

如果在一个 `CheckResponse` 周期内，车辆某个模块驾驶模块反馈处于接管或者手动驾驶模式，则 Apollo 会控制车辆使能为紧急停车模式（`Emergency`），即各模块均控制为手动模式，确保控制车辆时的安全。

不同的车辆 `CheckResponse` 周期可能不同，需要开发者根据情况通过设置 `retry_num` 设定 `check` 周期。

开发者可以不改原 check 代码方案，将 3 个驾驶模式反馈报文与 Apollo 内 `chassis_detail` 做映射：

`is_eps_online->转向模式反馈信号`

`is_vcu_online->驱动模式反馈信号`

`is_esp_online->制动模式反馈信号`

在 `apollo/modules/canbus/vehicle/ge3/protocol/scu_eps_311.cc` 文件内，增加以下代码：

```
chassis->mutable_check_response()->set_is_eps_online(eps_drvmode(bytes, length) == 3);
```

在 `apollo/modules/canbus/vehicle/ge3/protocol/scu_vcu_1_312.cc` 文件内，增加以下代码：

```
chassis->mutable_check_response()->set_is_vcu_online(vcu_drvmode(bytes, length) == 3);
```

在 `apollo/modules/canbus/vehicle/ge3/protocol/scu_bcs_1_306.cc` 文件内，增加以下代码：

```
chassis->mutable_check_response()->set_is_esp_online(bcs_drvmode(bytes, length) == 3);
```

`CheckResponse` 实现代码如下：

```
bool Ge3Controller::CheckResponse(const int32_t flags, bool need_wait) {
  int32_t retry_num = 20;
  ChassisDetail chassis_detail;
  bool is_eps_online = false;
  bool is_vcu_online = false;
  bool is_esp_online = false;

  do {
    if (message_manager_->GetSensorData(&chassis_detail) != ErrorCode::OK) {
      AERROR_EVERY(100) << "get chassis detail failed.";
      return false;
    }
    bool check_ok = true;
    if (flags & CHECK_RESPONSE_STEER_UNIT_FLAG) {
      is_eps_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_eps_online() &&
                      chassis_detail.check_response().is_eps_online();
      check_ok = check_ok && is_eps_online;
    }

    if (flags & CHECK_RESPONSE_SPEED_UNIT_FLAG) {
      is_vcu_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_vcu_online() &&
                      chassis_detail.check_response().is_vcu_online();
      is_esp_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_esp_online() &&
                      chassis_detail.check_response().is_esp_online();
      check_ok = check_ok && is_vcu_online && is_esp_online;
    }
    if (check_ok) {
      return true;
    }
    AINFO << "Need to check response again.";
    if (need_wait) {
      --retry_num;
      std::this_thread::sleep_for(
          std::chrono::duration<double, std::milli>(20));
    }
  } while (need_wait && retry_num);

  AINFO << "check_response fail: is_eps_online:" << is_eps_online
        << ", is_vcu_online:" << is_vcu_online
        << ", is_esp_online:" << is_esp_online;
  return false;
}
```

### 4. 修改底盘车速反馈协议

**重点：将车速反馈单位由 km/h 转化为 m/s **

Apollo 系统内默认使用车速反馈量为 `m/s`，底盘车速信息对 Apollo 非常重要，在车辆标定、控制、规划等都需要采集该数据，所以在开发适配代码时，要重点检查车速反馈的单位。车速由`km/h`转化为 `m/s` 时，在反馈车速的信号除以 `3.6` 即可。
找到 Ge3 车辆反馈车速的报文在文件 `apollo/modules/canbus/vehicle/ge3/protocol/scu_bcs_2_307.cc` 下，反馈车速消息为`Scubcs2307::bcs_vehspd{}`，如下图所示：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_9e7ea07.png)

### 5. 注册新车辆

1. 在 `modules/canbus/vehicle/vehicle_factory.cc` 里注册新的车辆，在该文件内新建如下类：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_2ecd09d.png)

2. 添加头文件：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_3a20644.png)

3. 添加 BUILD 依赖库，在 `apollo/modules/canbus/vehicle/BUILD` 文件内添加 `ge3_vehicle_factory` 依赖库。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_45e5eb6.png)

### 6. 更新配置文件

1. 在 `apollo/modules/common/configs/proto/vehicle_config.proto` 文件内添加 GE3 车辆分支。

![add_ge3_proto.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/add_ge3_proto_7a42ba6.png)

2. 在 `apollo/modules/canbus/conf/canbus_conf.pb.txt` 更新配置，改为 ge3 的 canbus 通信程序。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_1baf464.png)
