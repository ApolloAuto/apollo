// Copyright 2017 Baidu Inc. All Rights Reserved.
// Author: Chongchong Li (lichongchong@baidu.com)
//
// The ROS driver for Conti ARS408-21.

#include "conti_radar/conti_radar_base.h"
#include <malloc.h>

namespace apollo {
namespace drivers {
namespace conti_radar {

ContiRadarBase::ContiRadarBase(ros::Publisher& pub, int output_type,
                               std::string module_name)
    : pub_(pub), output_type_(output_type), module_name_(module_name) {}

bool ContiRadarBase::open() {
  if (init_device() && init_can()) {
    set_object_ids();
    return true;
  }   
  return false;
}

void ContiRadarBase::close() {
  close_device();
}

::adu::common::header::Header create_header(ros::Time time,
                                            const std::string& module_name,
                                            int sequence_num) {
  ::adu::common::header::Header header;
  header.set_timestamp_sec(time.toSec());
  header.set_radar_timestamp(time.toNsec());
  header.set_module_name(module_name);
  header.set_sequence_num(sequence_num);
}

::adu::common::header::Header create_header(timeval time,
                                            const std::string& module_name,
                                            int sequence_num) {
  ::adu::common::header::Header header;
  double second = (double)(time.tv_sec) + (double)(time.tv_usec) * 1e-6;
  uint64_t nanosecond = time.tv_sec * 1e9 + time.tv_usec * 1e3;
  header.set_timestamp_sec(second);
  header.set_radar_timestamp(nanosecond);
  header.set_module_name(module_name);
  header.set_sequence_num(sequence_num);
}

bool ContiRadarBase::communicate(int can_port, CanMsg* testcanframe,
                                 bool testflag) {
  CanMsg* canframebuf = new CanMsg[CAN_BUFFER_NUM];
  int can_num = 0;
  bool recv_flag = recv_can(canframebuf, &can_num, testcanframe, testflag);
  if (recv_flag == false) {
    if (recv_fail_cnt_ == 0) {
      ROS_ERROR("Power or CAN disconnection: reconnect within 1 min");
    } else if (recv_fail_cnt_ == DISCONNECT_WAIT) {
      delete[] canframebuf;
      return false;
    }
    ++recv_fail_cnt_;
    delete[] canframebuf;
    return true;
  }

  if (recv_fail_cnt_ > 0) {
    recv_fail_cnt_ = 0;
    init_can();
    ROS_INFO("Connection is well");
  }
  CanMsg* canframe;
  for (int canindex = 0; canindex < can_num; ++canindex) {
    canframe = canframebuf + canindex;
    uint32_t msgid = canframe->id;
    switch (msgid) {
      case CLUSTERSTA: {
        ::adu::common::header::Header header;
        double radartimesec = 0.0;
        uint64_t radartimensec = 0;
        if (scan_index_ >= 0) {
          header = create_header(ros::Time::now(), module_name_, scan_index_);
          object_msg_.mutable_header()->CopyFrom(header);
          std_msgs::String pubstring;
          if (!object_msg_.SerializeToString(&pubstring.data)) {
            ROS_ERROR("Failed to serialize radardata");
          }
          if (testflag == false) {
            pub_.publish(pubstring);
          }
          object_msg_.clear_contiobs();
          object_msg_.clear_header();
        }
        scan_index_ = (static_cast<unsigned int>((canframe->data)[2]) << 8) +
                      static_cast<unsigned int>((canframe->data)[3]);

        if (can_port >= 0) {
          header =
              create_header(canframe->timestamp, module_name_, scan_index_);
        } else {
          header = create_header(ros::Time::now(), module_name_, scan_index_)
        }
        object_msg_.set_measurement_time(radartimesec);
        object_msg_.mutable_header()->CopyFrom(header);
        object_msg_.set_type(::adu::common::sensor::CONTINENTAL_ARS_40821);
        detect_num_ = static_cast<int>((canframe->data)[0]) +
                      static_cast<int>((canframe->data)[1]);
        detect_index_ = 0;
        break;
      }
      case CLUSTER_GENGERAL INFO: {
        if (scan_index_ >= 0) {
          CLUSTER_GENGERAL INFOeral(canframe, can_port);
        }
        break;
      }
      case CLUSTER_QUALITY_INFO: {
        if (scan_index_ >= 0) {
          cluster_quality(canframe);
          ++detect_index_;
        }
        break;
      }
      case OBJECT_LIST_STATUS: {
        ::adu::common::header::Header headertemp;
        double radartimesec = 0.0;
        uint64_t radartimensec = 0;
        set_object_ids();
        if (scan_index_ >= 0) {
          header = create_header(ros::Time::now(), module_name_, scan_index_);
          object_msg_.mutable_header()->CopyFrom(headertemp);
          std_msgs::String pubstring;
          if (!object_msg_.SerializeToString(&pubstring.data)) {
            ROS_ERROR("Failed to serialize radardata");
          }
          pub_.publish(pubstring);
          double timedelay = radartimesec - object_msg_.measurement_time();
          ROS_INFO("delay time: %f\n", timedelay);
          object_msg_.clear_contiobs();
          object_msg_.clear_header();
        }
        scan_index_ = (static_cast<unsigned int>((canframe->data)[1]) << 8) +
                      static_cast<unsigned int>((canframe->data)[2]);
        if (can_port >= 0) {
          radartimesec = (double)(canframe->timestamp.tv_sec) +
                         (double)(canframe->timestamp.tv_usec) * 1e-6;
          radartimensec = canframe->timestamp.tv_sec * 1e9 +
                          canframe->timestamp.tv_usec * 1e3;
        } else {
          ros::Time nowtime = ros::Time::now();
          radartimesec = (double)nowtime.sec + (double)nowtime.nsec * 1e-9;
          radartimensec = nowtime.sec * 1e9 + nowtime.nsec;
        }
        object_msg_.set_measurement_time(radartimesec);
        headertemp.set_timestamp_sec();
        headertemp.set_radar_timestamp(radartimensec);
        headertemp.set_module_name(module_name_.c_str());
        headertemp.set_sequence_num(scan_index_);
        object_msg_.mutable_header()->CopyFrom(headertemp);
        object_msg_.set_type(::adu::common::sensor::CONTINENTAL_ARS_40821);

        detect_num_ = static_cast<uint32_t>((canframe->data)[0]);
        detect_index_ = 0;
        OBJECT_QUALITY_INFO_index_ = 0;
        break;
      }
      case OBJECT_GENERAL_INFO: {
        if (scan_index_ >= 0 && detect_index_ == 0 && OBJECT_QUALITY_INFO_index_ == 0 &&
            object_msg_.contiobs_size() < scan_index_) {
          track_general(canframe, can_port);
        }
        break;
      }
      case OBJECT_QUALITY_INFO: {
        if (scan_index_ >= 0) {
          track_quality(canframe);
          ++OBJECT_QUALITY_INFO_index_;
        }
        break;
      }
      case OBJECT_EXTENDED_INFO: {
        if (scan_index_ >= 0) {
          track_extend(canframe);
          ++detect_index_;
        }
        break;
      }
      case RADAR_STATE: {
        radar_state(canframe);
        break;
      }
      default:
        break;
    }
  }
  delete[] canframebuf;
  return true;
}

void ContiRadarBase::CLUSTER_GENGERAL INFOeral(CanMsg* canframe, int can_port) {
  uint32_t* buffer = new uint32_t[canframe->len];
  for (int byteindex = 0; byteindex < canframe->len; ++byteindex) {
    buffer[byteindex] = static_cast<uint32_t>(canframe->data[byteindex]);
  }
  ::adu::common::sensor::ContiRadarObs* object_temp;
  object_temp = object_msg_.add_contiobs();

  uint32_t longdist = ((buffer[1] & 0xFF) << 5) | ((buffer[2] & 0xF8) >> 3);
  uint32_t latdist = ((buffer[2] & 0x03) << 8) | buffer[3];
  uint32_t longvel = ((buffer[4] & 0xFF) << 2) | ((buffer[5] & 0xC0) >> 6);
  uint32_t latvel = ((buffer[5] & 0x3F) << 3) | ((buffer[6] & 0xE0) >> 5);

  bool clusterortrack = true;
  int obstacle_id = buffer[0];
  double longitude_dist = static_cast<double>(longdist) * 0.2 - 500.0;
  double lateral_dist = static_cast<double>(latdist) * 0.2 - 102.3;
  double longitude_vel = static_cast<double>(longvel) * 0.25 - 128.0;
  double lateral_vel = static_cast<double>(latvel) * 0.25 - 64.0;
  double rcs = static_cast<double>(buffer[7]) * 0.5 - 64.0;
  int dynprop = (buffer[6] & 0x07);

  object_temp->set_clusterortrack(clusterortrack);
  object_temp->set_obstacle_id(obstacle_id);
  object_temp->set_longitude_dist(longitude_dist);
  object_temp->set_lateral_dist(lateral_dist);
  object_temp->set_longitude_vel(longitude_vel);
  object_temp->set_lateral_vel(lateral_vel);
  object_temp->set_rcs(rcs);
  object_temp->set_dynprop(dynprop);

  ::adu::common::header::Header header;
  double radartimesec = 0.0;
  uint64_t radartimensec = 0;
  if (can_port >= 0) {
    header =
        create_header(canframe->timestamp, object_msg_.header().module_name(), object_msg_.header().sequence_num());
  } else {
    header = create_header(ros::Time::now(), object_msg_.header().module_name(), object_msg_.header().sequence_num());
  }
  object_temp->mutable_header()->CopyFrom(header);
  delete[] buffer;
}

void ContiRadarBase::cluster_quality(CanMsg* canframe) {
  uint32_t* buffer = new uint32_t[canframe->len];
  for (int byteindex = 0; byteindex < canframe->len; ++byteindex) {
    buffer[byteindex] = static_cast<uint32_t>(canframe->data[byteindex]);
  }
  uint32_t target_id = buffer[0];
  uint32_t invalidstate = (buffer[4] & 0xF8) >> 3;
  uint32_t ambigstate = buffer[4] & 0x07;
  uint32_t longdistrms = (buffer[1] & 0xF8) >> 3;
  uint32_t latdistrms = ((buffer[1] & 0x07) << 2) | ((buffer[2] & 0xC0) >> 6);
  uint32_t longvelrms = (buffer[2] & 0x3E) >> 1;
  uint32_t latvelrms = ((buffer[2] & 0x01) << 4) | ((buffer[3] & 0xF0) >> 4);
  uint32_t pdh0 = buffer[3] & 0x07;

  if (detect_index_ >= object_msg_.contiobs_size()) {
    delete[] buffer;
    return;
  }

  auto conti_obs = object_msg_.mutable_contiobs(detect_index_);
  conti_obs->set_longitude_dist_rms(LINEAR_RMS[longdistrms]);
  conti_obs->set_lateral_dist_rms(LINEAR_RMS[latdistrms]);
  conti_obs->set_longitude_vel_rms(LINEAR_RMS[longvelrms]);
  conti_obs->set_lateral_vel_rms(LINEAR_RMS[latvelrms]);
  conti_obs->set_probexist(PROBOFEXIST[pdh0]);

  if (invalidstate == 0x01 || invalidstate == 0x02 || invalidstate == 0x03 ||
      invalidstate == 0x06 || invalidstate == 0x07 || invalidstate == 0x0E ||
      ambigstate == 0x0 || ambigstate == 0x01 || ambigstate == 0x02) {
    conti_obs->set_probexist(PROBOFEXIST[0]);
  }
  delete[] buffer;
}

void ContiRadarBase::track_general(CanMsg* canframe, int can_port) {
  uint32_t* buffer = new uint32_t[canframe->len];
  for (int byteindex = 0; byteindex < canframe->len; ++byteindex) {
    buffer[byteindex] = static_cast<uint32_t>(canframe->data[byteindex]);
  }
  ::adu::common::sensor::ContiRadarObs* object_temp;
  object_temp = object_msg_.add_contiobs();

  uint32_t longdist = ((buffer[1] & 0xFF) << 5) | ((buffer[2] & 0xF8) >> 3);
  uint32_t latdist = ((buffer[2] & 0x07) << 8) | buffer[3];
  uint32_t longvel = ((buffer[4] & 0xFF) << 2) | ((buffer[5] & 0xC0) >> 6);
  uint32_t latvel = ((buffer[5] & 0x3F) << 3) | ((buffer[6] & 0xE0) >> 5);

  bool clusterortrack = false;
  int obstacle_id = buffer[0];
  object_ids_[obstacle_id] = object_msg_.contiobs_size() - 1;
  double longitude_dist = static_cast<double>(longdist) * 0.2 - 500.0;
  double lateral_dist = static_cast<double>(latdist) * 0.2 - 204.6;
  double longitude_vel = static_cast<double>(longvel) * 0.25 - 128.0;
  double lateral_vel = static_cast<double>(latvel) * 0.25 - 64.0;
  double rcs = static_cast<double>(buffer[7]) * 0.5 - 64.0;
  int dynprop = (buffer[6] & 0x07);

  object_temp->set_clusterortrack(clusterortrack);
  object_temp->set_obstacle_id(obstacle_id);
  object_temp->set_longitude_dist(longitude_dist);
  object_temp->set_lateral_dist(lateral_dist);
  object_temp->set_longitude_vel(longitude_vel);
  object_temp->set_lateral_vel(lateral_vel);
  object_temp->set_rcs(rcs);
  object_temp->set_dynprop(dynprop);

  ::adu::common::header::Header headertemp;
  double radartimesec = 0.0;
  uint64_t radartimensec = 0;
  if (can_port >= 0) {
    radartimesec = (double)(canframe->timestamp.tv_sec) +
                   (double)(canframe->timestamp.tv_usec) * 1e-6;
    radartimensec =
        canframe->timestamp.tv_sec * 1e9 + canframe->timestamp.tv_usec * 1e3;
  } else {
    ros::Time nowtime = ros::Time::now();
    radartimesec = (double)nowtime.sec + (double)nowtime.nsec * 1e-9;
    radartimensec = nowtime.sec * 1e9 + nowtime.nsec;
  }
  headertemp.set_timestamp_sec(radartimesec);
  headertemp.set_radar_timestamp(radartimensec);
  headertemp.set_module_name(object_msg_.header().module_name());
  headertemp.set_sequence_num(object_msg_.header().sequence_num());
  object_temp->mutable_header()->CopyFrom(headertemp);
  delete[] buffer;
}

void ContiRadarBase::track_quality(CanMsg* canframe) {
  uint32_t* buffer = new uint32_t[canframe->len];
  for (int byteindex = 0; byteindex < canframe->len; ++byteindex) {
    buffer[byteindex] = static_cast<uint32_t>(canframe->data[byteindex]);
  }

  uint32_t target_id = buffer[0];
  if (object_ids_[target_id] < 0) {
    delete[] buffer;
    return;
  }
  uint32_t longdistrms = (buffer[1] & 0xF8) >> 3;
  uint32_t latdistrms = ((buffer[1] & 0x07) << 2) | ((buffer[2] & 0xC0) >> 6);
  uint32_t longvelrms = (buffer[2] & 0x3E) >> 1;
  uint32_t latvelrms = ((buffer[2] & 0x01) << 4) | ((buffer[3] & 0xF0) >> 4);
  uint32_t longaccelrms = ((buffer[3] & 0x0F) << 1) | ((buffer[4] & 0x80) >> 7);
  uint32_t lataccelrms = (buffer[4] & 0x7C) >> 2;
  uint32_t oritationrms = ((buffer[4] & 0x03) << 3) | ((buffer[5] & 0xE0) >> 5);
  uint32_t pdh0 = (buffer[6] & 0xE0) >> 5;
  uint32_t meas_state = (buffer[6] & 0x1C) >> 2;

  auto conti_obs = object_msg_.mutable_contiobs(object_ids_[target_id]);
  conti_obs->set_longitude_dist_rms(LINEAR_RMS[longdistrms]);
  conti_obs->set_lateral_dist_rms(LINEAR_RMS[latdistrms]);
  conti_obs->set_longitude_vel_rms(LINEAR_RMS[longvelrms]);
  conti_obs->set_lateral_vel_rms(LINEAR_RMS[latvelrms]);
  conti_obs->set_longitude_accel_rms(LINEAR_RMS[longaccelrms]);
  conti_obs->set_lateral_accel_rms(LINEAR_RMS[lataccelrms]);
  conti_obs->set_oritation_angle_rms(ANGLE_RMS[oritationrms]);
  conti_obs->set_probexist(PROBOFEXIST[pdh0]);
  conti_obs->set_meas_state(meas_state);

  delete[] buffer;
}

void ContiRadarBase::track_extend(CanMsg* canframe) {
  uint32_t* buffer = new uint32_t[canframe->len];
  for (int byteindex = 0; byteindex < canframe->len; ++byteindex) {
    buffer[byteindex] = static_cast<uint32_t>(canframe->data[byteindex]);
  }

  uint32_t target_id = buffer[0];
  if (object_ids_[target_id] < 0) {
    delete[] buffer;
    return;
  }
  uint32_t longaccel = ((buffer[1] & 0xFF) << 3) | ((buffer[2] & 0xE0) >> 5);
  uint32_t lataccel = ((buffer[2] & 0x1F) << 4) | ((buffer[3] & 0xF0) >> 4);
  uint32_t objclass = buffer[3] & 0x07;
  uint32_t oritationangle =
      ((buffer[4] & 0xFF) << 2) | ((buffer[5] & 0xC0) >> 6);
  uint32_t objlength = buffer[6];
  uint32_t objwidth = buffer[7];
  double longitude_accel = static_cast<double>(longaccel) * 0.01 - 10.0;
  double lateral_accel = static_cast<double>(lataccel) * 0.01 - 2.50;
  double oritation_angle = static_cast<double>(oritationangle) * 0.4 - 180.0;
  double length = static_cast<double>(objlength) * 0.2;
  double width = static_cast<double>(objwidth) * 0.2;

  auto conti_obs = object_msg_.mutable_contiobs(object_ids_[target_id]);
  conti_obs->set_longitude_accel(longitude_accel);
  conti_obs->set_lateral_accel(lateral_accel);
  conti_obs->set_oritation_angle(oritation_angle);
  conti_obs->set_length(length);
  conti_obs->set_width(width);
  conti_obs->set_obstacle_class(objclass);
  delete[] buffer;
}

void ContiRadarBase::radar_state(CanMsg* canframe) {
  uint32_t* buffer = new uint32_t[canframe->len];
  for (int byteindex = 0; byteindex < canframe->len; ++byteindex) {
    buffer[byteindex] = static_cast<uint32_t>(canframe->data[byteindex]);
  }

  uint32_t max_dist = ((buffer[1] & 0xFF) << 2) | ((buffer[2] & 0xC0) >> 6);
  uint32_t radarpower = ((buffer[3] & 0x03) << 1) | ((buffer[4] & 0x80) >> 7);
  uint32_t output_type = (buffer[5] & 0x0C) >> 2;
  uint32_t rcs_threshold = (buffer[7] & 0x1C) >> 2;
  bool sendquality = true;
  bool sendextend = true;
  if (buffer[5] & 0x10) {
    sendquality = true;
  } else {
    sendquality = false;
  }
  if (buffer[5] & 0x20) {
    sendextend = true;
  } else {
    sendextend = false;
  }

  max_dist *= 2;
  int attengain = static_cast<int>(radarpower) * (-3);
  std::string cluster_track;
  if (output_type == 0x01) {
    cluster_track = "track";
  } else if (output_type = 0x02) {
    cluster_track = "cluster";
  } else {
    cluster_track = "error";
  }

  std::string rcs_th;
  if (rcs_threshold == 0x00) {
    rcs_th = "standard";
  } else if (rcs_threshold == 0x01) {
    rcs_th = "high sensitivity";
  } else {
    rcs_th = "error";
  }

  if (sendquality != send_quality_ || sendextend != send_extend_ ||
      max_dist != max_dist_ || attengain != attenuation_ ||
      output_type != output_type_ + 1 || rcs_threshold != rcs_threshold_) {
    canframe->id = 0x200;
    canframe->len = 8;
    canframe->data[0] = 0xB8;
    canframe->data[1] = 0x1F;
    canframe->data[2] = 0x40;
    canframe->data[3] = 0x00;
    if (output_type_ == 0) {
      canframe->data[4] = 0x08;
    } else {
      canframe->data[4] = 0x10;
    }
    canframe->data[5] = 0x8C;
    canframe->data[6] = 0x01;
    canframe->data[7] = 0x00;
    send_can(canframe, 1);
  }

  ROS_INFO(
      "distance:%dm, output:%s, attenuation:%ddB,"
      "threshold:%s, sendquality:%d, sendextend:%d\n",
      max_dist, cluster_track.c_str(), attengain, rcs_th.c_str(), sendquality,
      sendextend);
  delete[] buffer;
}
bool ContiRadarBase::init_can() {
  CanMsg* canframe = new CanMsg[CAN_BUFFER_NUM];
  detect_num_ = 0;
  detect_index_ = 0;
  OBJECT_QUALITY_INFO_index_ = 0;
  scan_index_ = -1;
  send_quality_ = true;
  send_extend_ = true;
  attenuation_ = 0;
  max_dist_ = 250;
  rcs_threshold_ = 0;
  int wait_time = 0;
  int can_num = 1;
  while ((wait_time++) < WAIT_TIME) {
    bool recv_flag = recv_can(canframe, &can_num);
    if (recv_flag == true) {
      bool state_flag = false;
      for (int canindex = 0; canindex < can_num; ++canindex) {
        if ((canframe + canindex)->id == RADAR_STATE) {
          radar_state(canframe);
          state_flag = true;
          break;
        }
      }
      if (state_flag) {
        break;
      }
    }
  }

  if (wait_time == WAIT_TIME) {
    ROS_ERROR(
        "%s: failed to receive data, device (%s) is disconnected? "
        "please connect it and restart the conti_radar_node\n",
        __FUNC__, get_device_info());
    return false;
  }

  wait_time = 0;
  can_num = 1;
  while ((wait_time++) < WAIT_TIME) {
    bool recv_flag = recv_can(canframe, &can_num);
    if (recv_flag == true) {
      bool state_flag = false;
      for (int canindex = 0; canindex < can_num; ++canindex) {
        if ((canframe + canindex)->id == RADAR_STATE) {
          radar_state(canframe);
          state_flag = true;
          break;
        }
      }
      if (state_flag) {
        break;
      }
    }
  }

  if (wait_time == WAIT_TIME) {
    ROS_ERROR("%s: device (%s), start failed", __FUNC__, get_device_info());
    return false;
  } else {
    ROS_INFO("%s: device (%s), start success", __FUNC__, get_device_info());
  }

  delete[] canframe;
  return true;
}

}
}
}
