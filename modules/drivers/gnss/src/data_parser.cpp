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

#include <cmath>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <Eigen/Geometry>
#include <proj_api.h>
#include <boost/array.hpp>

#include "gnss/parser.h"
#include "util/time_conversion.h"
#include "gnss/utils.h"
#include "data_parser.h"
#include "proto/gnss.pb.h"
#include "proto/imu.pb.h"
#include "proto/ins.pb.h"
#include "proto/gpgga.pb.h"

#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/gps.pb.h"

namespace apollo {
namespace drivers{
namespace gnss {

namespace {

constexpr double DEG_TO_RAD_LOCAL = M_PI / 180.0;
const std::string WGS84_TEXT = "+proj=latlong +ellps=WGS84";
const std::string UTM_TARGET = "+proj=utm +zone=10 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs ";

//covariance data for pose if can not get from novatel inscov topic
static const boost::array<double, 36> POSE_COVAR = {2, 0, 0, 0, 0, 0,
                                       0, 2, 0, 0, 0, 0,
                                       0, 0, 2, 0, 0, 0,
                                       0, 0, 0, 0.01, 0, 0,
                                       0, 0, 0, 0, 0.01, 0,
                                       0, 0, 0, 0, 0, 0.01
                                      };

template <class T>
void publish_message_raw(const ros::Publisher& pub, const T* pb) {
    std_msgs::String msg_pub;

    if (pb->SerializeToString(&msg_pub.data)) {
        pub.publish(msg_pub);
        return;
    }
    ROS_ERROR("Failed to serialize message.");
}

Parser* create_parser(config::Stream::Format format, bool is_base_station = false) {
    switch (format) {
    case config::Stream::NOVATEL_BINARY:
        return Parser::create_novatel();

    default:
        return nullptr;
    }
}

}  // namespace

DataParser::DataParser(ros::NodeHandle &nh, const std::string &raw_data_topic,
                       const std::string &gpgga_topic,
                       const std::string &corr_imu_topic, const std::string &odometry_topic,
                       const std::string &gnss_status_topic, const std::string &ins_status_topic) :
    _raw_data_sub(nh.subscribe(raw_data_topic, 256, &DataParser::raw_data_callback, this)),
    _gpgga_publisher(nh.advertise<std_msgs::String>(gpgga_topic, 64)),
    _imu_publisher(nh.advertise<apollo::localization::Imu>(corr_imu_topic, 64)),
    _nav_odometry_publisher(nh.advertise<apollo::localization::Gps>(odometry_topic, 64)),
    _gnss_status_publisher(nh.advertise<apollo::common::gnss_status::GnssStatus>(gnss_status_topic, 64, true)),
    _ins_status_publisher(nh.advertise<apollo::common::gnss_status::InsStatus>(ins_status_topic, 64, true)) {

    std::string utm_target_param;
    nh.param("proj4_text", utm_target_param, UTM_TARGET);
    ROS_INFO_STREAM("proj4_text : " << utm_target_param);

    _wgs84pj_source = pj_init_plus(WGS84_TEXT.c_str());
    _utm_target = pj_init_plus(utm_target_param.c_str());
    _gnss_status.reset(new apollo::common::gnss_status::GnssStatus());
    _ins_status.reset(new apollo::common::gnss_status::InsStatus());
    if (_gnss_status) {
        _gnss_status->set_solution_status(0);
        _gnss_status->set_num_sats(0);
        _gnss_status->set_position_type(0);
        _gnss_status->set_solution_completed(false);
    }

    if (_ins_status) {
        _ins_status->set_type(apollo::common::gnss_status::InsStatus::INVALID);
    }
}

bool DataParser::init(const std::string& cfg_file) {
    config::Config config;
    if ((!_ins_status) || (!_gnss_status)) {
        ROS_ERROR_STREAM("New ins status or gnss status failed.");
        return false;
    }
    _ins_status->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    _gnss_status->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    _ins_status_publisher.publish(_ins_status);
    _gnss_status_publisher.publish(_gnss_status);
    if (!parse_config_text(cfg_file, &config)) {
        ROS_FATAL_STREAM("Failed to load config file: " << cfg_file);
        return false;
    }

    ROS_INFO_STREAM("Creating data parser of format: " << config.data().format());
    _data_parser.reset(create_parser(config.data().format(), false));
    if (!_data_parser) {
        ROS_FATAL("Failed to create data parser.");
        return false;
    }

    _inited_flag = true;
    return true;
}

void DataParser::raw_data_callback(const std_msgs::String::ConstPtr& msg) {
    if (!_inited_flag) {
        return;
    }

    _data_parser->update(msg->data);
    Parser::MessageType type;
    MessagePtr msg_ptr;

    for (;;) {
        type = _data_parser->get_message(msg_ptr);
        if (type == Parser::MessageType::NONE) break;
        dispatch_message(type, msg_ptr);
    }
}

void DataParser::check_ins_status(::apollo::drivers::gnss::Ins *ins) {
    if (_ins_status_record != static_cast<uint32_t>(ins->type())) {
        _ins_status_record = static_cast<uint32_t>(ins->type());
        switch (ins->type()) {
        case apollo::drivers::gnss::Ins::GOOD:
            _ins_status->set_type(apollo::common::gnss_status::InsStatus::GOOD);
            break;

        case apollo::drivers::gnss::Ins::CONVERGING:
            _ins_status->set_type(apollo::common::gnss_status::InsStatus::CONVERGING);
            break;

        case apollo::drivers::gnss::Ins::INVALID:
        default:
            _ins_status->set_type(apollo::common::gnss_status::InsStatus::INVALID);
            break;
        }
        _ins_status->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
        _ins_status_publisher.publish(_ins_status);
    }
}

void DataParser::check_gnss_status(::apollo::drivers::gnss::Gnss *gnss) {
    _gnss_status->set_solution_status(static_cast<uint32_t>(gnss->solution_status()));
    _gnss_status->set_num_sats(static_cast<uint32_t>(gnss->num_sats()));
    _gnss_status->set_position_type(static_cast<uint32_t>(gnss->position_type()));

    if (static_cast<uint32_t>(gnss->solution_status()) == 0) {
        _gnss_status->set_solution_completed(true);
    } else {
        _gnss_status->set_solution_completed(false);
    }
    _gnss_status->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    _gnss_status_publisher.publish(_gnss_status);
}

void DataParser::dispatch_message(Parser::MessageType type, MessagePtr message) {
    std_msgs::String msg_pub;

    switch (type) {
    case Parser::MessageType::GNSS:
        check_gnss_status(As<::apollo::drivers::gnss::Gnss>(message));
        break;

    case Parser::MessageType::INS:
        check_ins_status(As<::apollo::drivers::gnss::Ins>(message));
        publish_corrimu_pb_message(message);
        publish_odometry_pb_message(message);
        break;

    case Parser::MessageType::GPGGA:
        publish_message_raw(_gpgga_publisher, As<::apollo::drivers::gnss::gpgga::GPGGA>(message));
    default:
        break;
    }
}

void DataParser::publish_odometry_pb_message(const MessagePtr message) {
    ::apollo::drivers::gnss::Ins *ins = As<::apollo::drivers::gnss::Ins>(message);
    boost::shared_ptr<::apollo::localization::Gps> gps(new ::apollo::localization::Gps());
    if (!gps) {
        ROS_ERROR("New gps failed.");
        return ;
    }

    double unix_sec = apollo::drivers::util::gps2unix(ins->measurement_time());
    gps->mutable_header()->set_timestamp_sec(unix_sec);
    auto *gps_msg = gps->mutable_localization();
    double pub_sec = ros::Time::now().toSec();
    ROS_DEBUG_STREAM("gps timeline odometry delay: " << pub_sec - unix_sec << " s.");

    // 1. pose xyz, TODO: 
    double x = ins->position().lon();;
    double y = ins->position().lat();
    x *= DEG_TO_RAD_LOCAL;
    y *= DEG_TO_RAD_LOCAL;

    int ret = pj_transform(_wgs84pj_source, _utm_target, 1, 1, &x, &y, NULL);
    if (ret != 0) {
        ROS_ERROR_STREAM("prj transform failed, x: " << x << ", y: " << y
                         << ", ret: " << ret);
        return;
    }

    gps_msg->mutable_position()->set_x(x);
    gps_msg->mutable_position()->set_y(y);
    gps_msg->mutable_position()->set_z(ins->position().height());

    // 2. orientation, TODO: check
    Eigen::Quaterniond q =
                Eigen::AngleAxisd(ins->euler_angles().z() - 90 * DEG_TO_RAD_LOCAL, Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(-ins->euler_angles().y(), Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(ins->euler_angles().x(), Eigen::Vector3d::UnitY());

    gps_msg->mutable_orientation()->set_qx(q.x());
    gps_msg->mutable_orientation()->set_qy(q.y());
    gps_msg->mutable_orientation()->set_qz(q.z());
    gps_msg->mutable_orientation()->set_qw(q.w());

    gps_msg->mutable_linear_velocity()->set_x(ins->linear_velocity().x());
    gps_msg->mutable_linear_velocity()->set_y(ins->linear_velocity().y());
    gps_msg->mutable_linear_velocity()->set_z(ins->linear_velocity().z());

    gps_msg->mutable_angular_velocity()->set_x(ins->angular_velocity().x());
    gps_msg->mutable_angular_velocity()->set_y(ins->angular_velocity().y());
    gps_msg->mutable_angular_velocity()->set_z(ins->angular_velocity().z());

    ROS_DEBUG_STREAM("local timeline odometry delay: " << (ros::Time::now().toSec() - ins->mutable_header()->timestamp_sec()) << " s.");
    _nav_odometry_publisher.publish(gps);
}

void DataParser::publish_corrimu_pb_message(const MessagePtr message) {
    ::apollo::drivers::gnss::Ins *ins = As<::apollo::drivers::gnss::Ins>(message);
    boost::shared_ptr<::apollo::localization::Imu> imu(new ::apollo::localization::Imu());
    if (!imu) {
        ROS_ERROR("New imu failed.");
        return ;
    }
    double unix_sec = apollo::drivers::util::gps2unix(ins->measurement_time());
    imu->mutable_header()->set_timestamp_sec(unix_sec);
    double pub_sec = ros::Time::now().toSec();
    ROS_DEBUG_STREAM("gps timeline imu delay: " << pub_sec - unix_sec << " s.");

    auto *imu_msg = imu->mutable_imu();
    imu_msg->mutable_linear_acceleration()->set_x(-ins->linear_acceleration().y());
    imu_msg->mutable_linear_acceleration()->set_y(ins->linear_acceleration().x());
    imu_msg->mutable_linear_acceleration()->set_z(ins->linear_acceleration().z());

    imu_msg->mutable_angular_velocity()->set_x(-ins->angular_velocity().y());
    imu_msg->mutable_angular_velocity()->set_y(ins->angular_velocity().x());
    imu_msg->mutable_angular_velocity()->set_z(ins->angular_velocity().z());

    ROS_DEBUG_STREAM("local timeline imu delay: " << (ros::Time::now().toSec() - ins->mutable_header()->timestamp_sec()) << " s.");
    _imu_publisher.publish(imu);
}

}
}
}
