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

// An parser for decoding binary messages from a NovAtel receiver. The following messages must be
// logged in order for this parser to work properly.
//
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include "gnss/parser.h"
#include "gnss/novatel_messages.h"
#include "proto/gnss.pb.h"
#include "proto/imu.pb.h"
#include "proto/ins.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {

// Anonymous namespace that contains helper constants and functions.
namespace {

constexpr size_t BUFFER_SIZE = 256;

constexpr int SECONDS_PER_WEEK = 60 * 60 * 24 * 7;

constexpr double DEG_TO_RAD = M_PI / 180.0;

constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();

// The NovAtel's orientation covariance matrix is pitch, roll, and yaw. We use the index array below
// to convert it to the orientation covariance matrix with order roll, pitch, and yaw.
constexpr int INDEX[] = {4, 3, 5,
                         1, 0, 2,
                         7, 6, 8};
static_assert(sizeof(INDEX) == 9 * sizeof(int), "Incorrect size of INDEX");

template <typename T>
constexpr bool is_zero(T value) {
    return value == static_cast<T>(0);
}

// CRC algorithm from the NovAtel document.
inline uint32_t crc32_word(uint32_t word) {
    for (int j = 0; j < 8; ++j) {
        if (word & 1) {
            word = (word >> 1) ^ 0xEDB88320;
        } else {
            word >>= 1;
        }
    }
    return word;
}

inline uint32_t crc32_block(const uint8_t* buffer, size_t length) {
    uint32_t word = 0;
    while (length--) {
        uint32_t t1 = (word >> 8) & 0xFFFFFF;
        uint32_t t2 = crc32_word((word ^ *buffer++) & 0xFF);
        word = t1 ^ t2;
    }
    return word;
}

// Converts NovAtel's azimuth (north = 0, east = 90) to FLU yaw (east = 0, north = pi/2).
constexpr double azimuth_deg_to_yaw_rad(double azimuth) {
    return (90.0 - azimuth) * DEG_TO_RAD;
}

// A helper that fills an Point3D object (which uses the FLU frame) using RFU measurements.
inline void rfu_to_flu(double r, double f, double u, ::apollo::common::Point3D* flu) {
    flu->set_x(f);
    flu->set_y(-r);
    flu->set_z(u);
}

}  // namespace

class NovatelParser : public Parser {
public:
    NovatelParser();

    virtual MessageType get_message(MessagePtr& message_ptr);

private:
    bool check_crc();

    Parser::MessageType prepare_message(MessagePtr& message_ptr);

    // The handle_xxx functions return whether a message is ready.
    bool handle_best_pos(const novatel::BestPos* pos, uint16_t gps_week, uint32_t gps_millisecs);

    bool handle_best_vel(const novatel::BestVel* vel, uint16_t gps_week, uint32_t gps_millisecs);

    bool handle_corr_imu_data(const novatel::CorrImuData* imu);

    bool handle_ins_cov(const novatel::InsCov* cov);

    bool handle_ins_pva(const novatel::InsPva* pva);

    bool handle_raw_imu_x(const novatel::RawImuX* imu);

    double _gyro_scale = 0.0;

    double _accel_scale = 0.0;

    float _imu_measurement_span = 1.0 / 200.0;
    float _imu_measurement_hz = 200.0;

    // TODO: Get mapping from configuration file.
    int _imu_frame_mapping = 5;

    double _imu_measurement_time_previous = -1.0;

    std::vector<uint8_t> _buffer;

    size_t _header_length = 0;

    size_t _total_length = 0;

    // -1 is an unused value.
    novatel::SolutionStatus _solution_status = static_cast<novatel::SolutionStatus>(-1);
    novatel::SolutionType _position_type = static_cast<novatel::SolutionType>(-1);
    novatel::SolutionType _velocity_type = static_cast<novatel::SolutionType>(-1);
    novatel::InsStatus _ins_status = static_cast<novatel::InsStatus>(-1);

    ::apollo::drivers::gnss::Gnss _gnss;
    ::apollo::drivers::gnss::Imu _imu;
    ::apollo::drivers::gnss::Ins _ins;
};

Parser* Parser::create_novatel() {
    return new NovatelParser();
}

NovatelParser::NovatelParser() {
    _buffer.reserve(BUFFER_SIZE);
    _ins.mutable_position_covariance()->Resize(9, FLOAT_NAN);
    _ins.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
    _ins.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);
}

Parser::MessageType NovatelParser::get_message(MessagePtr& message_ptr) {
    if (_data == nullptr) {
        return MessageType::NONE;
    }

    while (_data < _data_end) {
        if (_buffer.size() == 0) {  // Looking for SYNC0
            if (*_data == novatel::SYNC_0) {
                _buffer.push_back(*_data);
            }
            ++_data;
        } else if (_buffer.size() == 1) {  // Looking for SYNC1
            if (*_data == novatel::SYNC_1) {
                _buffer.push_back(*_data++);
            } else {
                _buffer.clear();
            }
        } else if (_buffer.size() == 2) {  // Looking for SYNC2
            switch (*_data) {
            case novatel::SYNC_2_LONG_HEADER:
                _buffer.push_back(*_data++);
                _header_length = sizeof(novatel::LongHeader);
                break;
            case novatel::SYNC_2_SHORT_HEADER:
                _buffer.push_back(*_data++);
                _header_length = sizeof(novatel::ShortHeader);
                break;
            default:
                _buffer.clear();
            }
        } else if (_header_length > 0) {  // Working on header.
            if (_buffer.size() < _header_length) {
                _buffer.push_back(*_data++);
            } else {
                if (_header_length == sizeof(novatel::LongHeader)) {
                    _total_length = _header_length + novatel::CRC_LENGTH +
                            reinterpret_cast<novatel::LongHeader*>(_buffer.data())->message_length;
                } else if (_header_length == sizeof(novatel::ShortHeader)) {
                    _total_length = _header_length + novatel::CRC_LENGTH +
                            reinterpret_cast<novatel::ShortHeader*>(_buffer.data())->message_length;
                } else {
                    ROS_ERROR("Incorrect _header_length. Should never reach here.");
                    _buffer.clear();
                }
                _header_length = 0;
            }
        } else if (_total_length > 0) {
            if (_buffer.size() < _total_length) {  // Working on body.
                _buffer.push_back(*_data++);
                continue;
            }
            MessageType type = prepare_message(message_ptr);
            _buffer.clear();
            _total_length = 0;
            if (type != MessageType::NONE) {
                return type;
            }
        }
    }
    return MessageType::NONE;
}

bool NovatelParser::check_crc() {
    size_t l = _buffer.size() - novatel::CRC_LENGTH;
    return crc32_block(_buffer.data(), l) == *reinterpret_cast<uint32_t*>(_buffer.data() + l);
}

Parser::MessageType NovatelParser::prepare_message(MessagePtr& message_ptr) {
    if (!check_crc()) {
        ROS_ERROR("CRC check failed.");
        return MessageType::NONE;
    }

    uint8_t* message = nullptr;
    novatel::MessageId message_id;
    uint16_t message_length;
    uint16_t gps_week;
    uint32_t gps_millisecs;
    if (_buffer[2] == novatel::SYNC_2_LONG_HEADER) {
        auto header = reinterpret_cast<const novatel::LongHeader*>(_buffer.data());
        message = _buffer.data() + sizeof(novatel::LongHeader);
        gps_week = header->gps_week;
        gps_millisecs = header->gps_millisecs;
        message_id = header->message_id;
        message_length = header->message_length;
    } else {
        auto header = reinterpret_cast<const novatel::ShortHeader*>(_buffer.data());
        message = _buffer.data() + sizeof(novatel::ShortHeader);
        gps_week = header->gps_week;
        gps_millisecs = header->gps_millisecs;
        message_id = header->message_id;
        message_length = header->message_length;
    }
    switch (message_id) {
    case novatel::BESTGNSSPOS:
    case novatel::BESTPOS:
    case novatel::PSRPOS:
        //ROS_ERROR_COND(message_length != sizeof(novatel::BestPos), "Incorrect message_length");
        if (message_length != sizeof(novatel::BestPos)) {
            ROS_ERROR("Incorrect message_length");
            break;
        }
        if (handle_best_pos(reinterpret_cast<novatel::BestPos*>(message), gps_week,
                            gps_millisecs)) {
            message_ptr = &_gnss;
            return MessageType::GNSS;
        }
        break;

    case novatel::BESTGNSSVEL:
    case novatel::BESTVEL:
    case novatel::PSRVEL:
        //ROS_ERROR_COND(message_length != sizeof(novatel::BestVel), "Incorrect message_length");
        if (message_length != sizeof(novatel::BestVel)) {
            ROS_ERROR("Incorrect message_length");
            break;
        }
        if (handle_best_vel(reinterpret_cast<novatel::BestVel*>(message), gps_week,
                            gps_millisecs)) {
            message_ptr = &_gnss;
            return MessageType::GNSS;
        }
        break;

    case novatel::CORRIMUDATA:
    case novatel::CORRIMUDATAS:
        //ROS_ERROR_COND(message_length != sizeof(novatel::CorrImuData), "Incorrect message_length");
        if (message_length != sizeof(novatel::CorrImuData)) {
            ROS_ERROR("Incorrect message_length");
            break;
        }

        if (handle_corr_imu_data(reinterpret_cast<novatel::CorrImuData*>(message))) {
            message_ptr = &_ins;
            return MessageType::INS;
        }
        break;

    case novatel::INSCOV:
    case novatel::INSCOVS:
        //ROS_ERROR_COND(message_length != sizeof(novatel::InsCov), "Incorrect message_length");
        if (message_length != sizeof(novatel::InsCov)) {
            ROS_ERROR("Incorrect message_length");
            break;
        }

        if (handle_ins_cov(reinterpret_cast<novatel::InsCov*>(message))) {
            message_ptr = &_ins;
            return MessageType::INS;
        }
        break;

    case novatel::INSPVA:
    case novatel::INSPVAS:
        //ROS_ERROR_COND(message_length != sizeof(novatel::InsPva), "Incorrect message_length");
        if (message_length != sizeof(novatel::InsPva)) {
            ROS_ERROR("Incorrect message_length");
            break;
        }

        if (handle_ins_pva(reinterpret_cast<novatel::InsPva*>(message))) {
            message_ptr = &_ins;
            return MessageType::INS;
        }
        break;

    case novatel::RAWIMUX:
    case novatel::RAWIMUSX:
        //ROS_ERROR_COND(message_length != sizeof(novatel::RawImuX), "Incorrect message_length");
        if (message_length != sizeof(novatel::RawImuX)) {
            ROS_ERROR("Incorrect message_length");
            break;
        }

        if (handle_raw_imu_x(reinterpret_cast<novatel::RawImuX*>(message))) {
            message_ptr = &_imu;
            return MessageType::IMU;
        }
        break;

    default:
        break;
    }
    return MessageType::NONE;
}

bool NovatelParser::handle_best_pos(const novatel::BestPos* pos, uint16_t gps_week, uint32_t gps_millisecs) {
    _gnss.mutable_position()->set_lon(pos->longitude);
    _gnss.mutable_position()->set_lat(pos->latitude);
    _gnss.mutable_position()->set_height(pos->height_msl + pos->undulation);
    _gnss.mutable_position_std_dev()->set_x(pos->longitude_std_dev);
    _gnss.mutable_position_std_dev()->set_y(pos->latitude_std_dev);
    _gnss.mutable_position_std_dev()->set_z(pos->height_std_dev);
    _gnss.set_num_sats(pos->num_sats_in_solution);
    if (_solution_status != pos->solution_status) {
        _solution_status = pos->solution_status;
        ROS_INFO_STREAM("Solution status: " << static_cast<int>(_solution_status));
    }
    if (_position_type != pos->position_type) {
        _position_type = pos->position_type;
        ROS_INFO_STREAM("Position type: " << static_cast<int>(_position_type));
    }
    _gnss.set_solution_status(static_cast<uint32_t>(pos->solution_status));
    if (pos->solution_status == novatel::SolutionStatus::SOL_COMPUTED) {
        _gnss.set_position_type(static_cast<uint32_t>(pos->position_type));
        switch (pos->position_type) {
        case novatel::SolutionType::SINGLE:
        case novatel::SolutionType::INS_PSRSP:
            _gnss.set_type(apollo::drivers::gnss::Gnss::SINGLE);
            break;
        case novatel::SolutionType::PSRDIFF:
        case novatel::SolutionType::WAAS:
        case novatel::SolutionType::INS_SBAS:
            _gnss.set_type(apollo::drivers::gnss::Gnss::PSRDIFF);
            break;
        case novatel::SolutionType::FLOATCONV:
        case novatel::SolutionType::L1_FLOAT:
        case novatel::SolutionType::IONOFREE_FLOAT:
        case novatel::SolutionType::NARROW_FLOAT:
        case novatel::SolutionType::RTK_DIRECT_INS:
        case novatel::SolutionType::INS_RTKFLOAT:
            _gnss.set_type(apollo::drivers::gnss::Gnss::RTK_FLOAT);
            break;
        case novatel::SolutionType::WIDELANE:
        case novatel::SolutionType::NARROWLANE:
        case novatel::SolutionType::L1_INT:
        case novatel::SolutionType::WIDE_INT:
        case novatel::SolutionType::NARROW_INT:
        case novatel::SolutionType::INS_RTKFIXED:
            _gnss.set_type(apollo::drivers::gnss::Gnss::RTK_INTEGER);
            break;
        case novatel::SolutionType::OMNISTAR:
        case novatel::SolutionType::INS_OMNISTAR:
        case novatel::SolutionType::INS_OMNISTAR_HP:
        case novatel::SolutionType::INS_OMNISTAR_XP:
        case novatel::SolutionType::OMNISTAR_HP:
        case novatel::SolutionType::OMNISTAR_XP:
        case novatel::SolutionType::PPP_CONVERGING:
        case novatel::SolutionType::PPP:
        case novatel::SolutionType::INS_PPP_CONVERGING:
        case novatel::SolutionType::INS_PPP:
            _gnss.set_type(apollo::drivers::gnss::Gnss::PPP);
            break;
        case novatel::SolutionType::PROPOGATED:
            _gnss.set_type(apollo::drivers::gnss::Gnss::PROPAGATED);
            break;
        default:
            _gnss.set_type(apollo::drivers::gnss::Gnss::INVALID);
        }
    } else {
        _gnss.set_type(apollo::drivers::gnss::Gnss::INVALID);
        _gnss.set_position_type(0);
    }
    if (pos->datum_id != novatel::DatumId::WGS84) {
        ROS_ERROR_STREAM_THROTTLE(5, "Unexpected Datum Id: " << static_cast<int>(pos->datum_id));
    }

    double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
    if (_gnss.measurement_time() != seconds) {
        _gnss.set_measurement_time(seconds);
        return false;
    }
    return true;
}

bool NovatelParser::handle_best_vel(const novatel::BestVel* vel, uint16_t gps_week, uint32_t gps_millisecs) {
    if (_velocity_type != vel->velocity_type) {
        _velocity_type = vel->velocity_type;
        ROS_INFO_STREAM("Velocity type: " << static_cast<int>(_velocity_type));
    }
    if (!_gnss.has_velocity_latency() || _gnss.velocity_latency() != vel->latency) {
        ROS_INFO_STREAM("Velocity latency: " << static_cast<int>(vel->latency));
        _gnss.set_velocity_latency(vel->latency);
    }
    double yaw = azimuth_deg_to_yaw_rad(vel->track_over_ground);
    _gnss.mutable_linear_velocity()->set_x(vel->horizontal_speed * cos(yaw));
    _gnss.mutable_linear_velocity()->set_y(vel->horizontal_speed * sin(yaw));
    _gnss.mutable_linear_velocity()->set_z(vel->vertical_speed);

    double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
    if (_gnss.measurement_time() != seconds) {
        _gnss.set_measurement_time(seconds);
        return false;
    }
    return true;
}

bool NovatelParser::handle_corr_imu_data(const novatel::CorrImuData* imu) {
    rfu_to_flu(imu->x_velocity_change * _imu_measurement_hz,
               imu->y_velocity_change * _imu_measurement_hz,
               imu->z_velocity_change * _imu_measurement_hz,
               _ins.mutable_linear_acceleration());
    rfu_to_flu(imu->x_angle_change * _imu_measurement_hz,
               imu->y_angle_change * _imu_measurement_hz,
               imu->z_angle_change * _imu_measurement_hz,
               _ins.mutable_angular_velocity());

    double seconds = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
    if (_ins.measurement_time() != seconds) {
        _ins.set_measurement_time(seconds);
        return false;
    }
    _ins.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    return true;
}

bool NovatelParser::handle_ins_cov(const novatel::InsCov* cov) {
    for (int i = 0; i < 9; ++i) {
        _ins.set_position_covariance(i, cov->position_covariance[i]);
        _ins.set_euler_angles_covariance(INDEX[i],
                                         (DEG_TO_RAD * DEG_TO_RAD) * cov->attitude_covariance[i]);
        _ins.set_linear_velocity_covariance(i, cov->velocity_covariance[i]);
    }
    return false;
}

bool NovatelParser::handle_ins_pva(const novatel::InsPva* pva) {
    if (_ins_status != pva->status) {
        _ins_status = pva->status;
        ROS_INFO_STREAM("INS status: " << static_cast<int>(_ins_status));
    }
    _ins.mutable_position()->set_lon(pva->longitude);
    _ins.mutable_position()->set_lat(pva->latitude);
    _ins.mutable_position()->set_height(pva->height);
    _ins.mutable_euler_angles()->set_x(pva->roll * DEG_TO_RAD);
    _ins.mutable_euler_angles()->set_y(-pva->pitch * DEG_TO_RAD);
    _ins.mutable_euler_angles()->set_z(azimuth_deg_to_yaw_rad(pva->azimuth));
    _ins.mutable_linear_velocity()->set_x(pva->east_velocity);
    _ins.mutable_linear_velocity()->set_y(pva->north_velocity);
    _ins.mutable_linear_velocity()->set_z(pva->up_velocity);

    switch (pva->status) {
    case novatel::InsStatus::ALIGNMENT_COMPLETE:
    case novatel::InsStatus::SOLUTION_GOOD:
        _ins.set_type(apollo::drivers::gnss::Ins::GOOD);
        break;
    case novatel::InsStatus::ALIGNING:
    case novatel::InsStatus::HIGH_VARIANCE:
    case novatel::InsStatus::SOLUTION_FREE:
        _ins.set_type(apollo::drivers::gnss::Ins::CONVERGING);
        break;
    default:
        _ins.set_type(apollo::drivers::gnss::Ins::INVALID);
    }

    double seconds = pva->gps_week * SECONDS_PER_WEEK + pva->gps_seconds;
    if (_ins.measurement_time() != seconds) {
        _ins.set_measurement_time(seconds);
        return false;
    }

    _ins.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    return true;
}

bool NovatelParser::handle_raw_imu_x(const novatel::RawImuX* imu) {
    if (imu->imu_error != 0) {
        ROS_WARN_STREAM("IMU error. Status: " << std::hex << std::showbase << imu->imuStatus);
    }
    if (is_zero(_gyro_scale)) {
        novatel::ImuParameter param = novatel::get_imu_parameter(imu->imu_type);
        ROS_INFO_STREAM("IMU type: " << static_cast<unsigned>(imu->imu_type) << "; "
                  << "Gyro scale: " << param.gyro_scale << "; "
                  << "Accel scale: " << param.accel_scale << "; "
                  << "Sampling rate: " << param.sampling_rate_hz << ".");
      
        if (is_zero(param.sampling_rate_hz)) {
            ROS_ERROR_STREAM_THROTTLE(5, "Unsupported IMU type: " << static_cast<int>(imu->imu_type));
            return false;
        }
        _gyro_scale = param.gyro_scale * param.sampling_rate_hz;
        _accel_scale = param.accel_scale * param.sampling_rate_hz;
        _imu_measurement_hz = param.sampling_rate_hz;
        _imu_measurement_span = 1.0 / param.sampling_rate_hz;
        _imu.set_measurement_span(_imu_measurement_span);
    }

    double time = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
    if (_imu_measurement_time_previous > 0.0 &&
        fabs(time - _imu_measurement_time_previous - _imu_measurement_span) > 1e-4) {
        ROS_WARN_STREAM("Unexpected delay between two IMU measurements at: "
                     << time - _imu_measurement_time_previous);
    }
    _imu.set_measurement_time(time);
    switch (_imu_frame_mapping) {
    case 5:  // Default mapping.
        rfu_to_flu(imu->x_velocity_change * _accel_scale,
                   -imu->y_velocity_change_neg * _accel_scale,
                   imu->z_velocity_change * _accel_scale,
                   _imu.mutable_linear_acceleration());
        rfu_to_flu(imu->x_angle_change * _gyro_scale,
                   -imu->y_angle_change_neg * _gyro_scale,
                   imu->z_angle_change * _gyro_scale,
                   _imu.mutable_angular_velocity());
        break;
    case 6:
        rfu_to_flu(-imu->y_velocity_change_neg * _accel_scale,
                   imu->x_velocity_change * _accel_scale,
                   -imu->z_velocity_change * _accel_scale,
                   _imu.mutable_linear_acceleration());
        rfu_to_flu(-imu->y_angle_change_neg * _gyro_scale,
                   imu->x_angle_change * _gyro_scale,
                   -imu->z_angle_change * _gyro_scale,
                   _imu.mutable_angular_velocity());
        break;
    default:
        ROS_ERROR_STREAM_THROTTLE(5, "Unsupported IMU frame mapping: " << _imu_frame_mapping);
    }
    _imu_measurement_time_previous = time;
    return true;
}


}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
