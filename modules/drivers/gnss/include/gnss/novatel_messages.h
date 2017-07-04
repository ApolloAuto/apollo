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

// This defines enums and structures for parsing NovAtel binary messages. Please refer to NovAtel's
// documents for details about these messages.
//  http://www.novatel.com/assets/Documents/Manuals/om-20000129.pdf
//  http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf

#ifndef DRIVERS_GNSS_NOVATEL_MESSAGES_H
#define DRIVERS_GNSS_NOVATEL_MESSAGES_H

#include <stdint.h>

namespace apollo {
namespace drivers {
namespace gnss {
namespace novatel {

enum MessageId : uint16_t {
    BESTGNSSPOS = 1429,
    BESTGNSSVEL = 1430,
    BESTPOS = 42,
    BESTVEL = 99,
    CORRIMUDATA = 812,
    CORRIMUDATAS = 813,
    INSCOV = 264,
    INSCOVS = 320,
    INSPVA = 507,
    INSPVAS = 508,
    INSPVAX = 1465,
    PSRPOS = 47,
    PSRVEL = 100,
    RAWIMU = 268,
    RAWIMUX = 1461,
    RAWIMUSX = 1462,
    MARK1PVA = 1067,
    GPGGA = 218,
};

// Every binary message has 32-bit CRC performed on all data including the header.
constexpr uint16_t CRC_LENGTH = 4;

#pragma pack(push, 1) // Turn off struct padding.

enum SyncByte : uint8_t {
    SYNC_0 = 0xAA,
    SYNC_1 = 0x44,
    SYNC_2_LONG_HEADER = 0x12,
    SYNC_2_SHORT_HEADER = 0x13,
};

struct MessageType {
    enum MessageFormat {
        BINARY = 0b00,
        ASCII = 0b01,
        ABREVIATED_ASCII = 0b10,
        NMEA = 0b11,
    };

    enum ResponseBit {
        ORIGINAL_MESSAGE = 0b0,
        RESPONSE_MESSAGE = 0b1,
    };

    uint8_t reserved : 5;
    MessageFormat format : 2;
    ResponseBit response : 1;
};

struct LongHeader {
    SyncByte sync[3];
    uint8_t header_length;
    MessageId message_id;
    MessageType message_type;
    uint8_t port_address;  // Address of the data port the log was received on.
    uint16_t message_length;  // Message length (not including the header nor the CRC).
    uint16_t sequence;  // Counts down from N-1 to 0 for multiple related logs.
    uint8_t idle;  // Time the processor was idle in last second between logs with same ID.
    uint8_t time_status;  // Indicates the quality of the GPS time.
    uint16_t gps_week;  // GPS Week number.
    uint32_t gps_millisecs;  // Milliseconds of week.
    uint32_t status;  // Receiver status.
    uint16_t reserved;
    uint16_t version;  // Receiver software build number.
};
static_assert(sizeof(LongHeader) == 28, "Incorrect size of LongHeader");

struct ShortHeader {
    SyncByte sync[3];
    uint8_t message_length; // Message length (not including the header nor the CRC).
    MessageId message_id;
    uint16_t gps_week;  // GPS Week number.
    uint32_t gps_millisecs;  // Milliseconds of week.
};
static_assert(sizeof(ShortHeader) == 12, "Incorrect size of ShortHeader");

enum class SolutionStatus : uint32_t {
    SOL_COMPUTED = 0,  // solution computed
    INSUFFICIENT_OBS,  // insufficient observations
    NO_CONVERGENCE,  // no convergence
    SINGULARITY,  // singularity at parameters matrix
    COV_TRACE,  // covariance trace exceeds maximum (trace > 1000 m)
    TEST_DIST,  // test distance exceeded (max of 3 rejections if distance > 10 km)
    COLD_START,  // not yet converged from cold start
    V_H_LIMIT,  // height or velocity limits exceeded
    VARIANCE,  // variance exceeds limits
    RESIDUALS,  // residuals are too large
    INTEGRITY_WARNING = 13,  // large residuals make position questionable
    PENDING = 18,  // receiver computes its position and determines if the fixed position is valid
    INVALID_FIX = 19,  // the fixed position entered using the fix position command is invalid
    UNAUTHORIZED = 20,  // position type is unauthorized
    INVALID_RATE = 22,  // selected logging rate is not supported for this solution type
};

enum class SolutionType : uint32_t {
    NONE = 0,
    FIXEDPOS = 1,
    FIXEDHEIGHT = 2,
    FLOATCONV = 4,
    WIDELANE = 5,
    NARROWLANE = 6,
    DOPPLER_VELOCITY = 8,
    SINGLE = 16,
    PSRDIFF = 17,
    WAAS = 18,
    PROPOGATED = 19,
    OMNISTAR = 20,
    L1_FLOAT = 32,
    IONOFREE_FLOAT = 33,
    NARROW_FLOAT = 34,
    L1_INT = 48,
    WIDE_INT = 49,
    NARROW_INT = 50,
    RTK_DIRECT_INS = 51,  // RTK filter is directly initialized from the INS filter.
    INS_SBAS = 52,
    INS_PSRSP = 53,
    INS_PSRDIFF = 54,
    INS_RTKFLOAT = 55,
    INS_RTKFIXED = 56,
    INS_OMNISTAR = 57,
    INS_OMNISTAR_HP = 58,
    INS_OMNISTAR_XP = 59,
    OMNISTAR_HP = 64,
    OMNISTAR_XP = 65,
    PPP_CONVERGING = 68,
    PPP = 69,
    INS_PPP_CONVERGING = 73,
    INS_PPP = 74,
};

enum class DatumId : uint32_t {
    // We only use WGS-84.
    WGS84 = 61,
};

struct BestPos {
    SolutionStatus solution_status;
    SolutionType position_type;
    double latitude;  // in degrees
    double longitude;  // in degrees
    double height_msl;  // height above mean sea level in meters
    float undulation;  // undulation = height_wgs84 - height_msl
    DatumId datum_id;  // datum id number
    float latitude_std_dev;  // latitude standard deviation (m)
    float longitude_std_dev; // longitude standard deviation (m)
    float height_std_dev;  // height standard deviation (m)
    char base_station_id[4];  // base station id
    float differential_age;  // differential position age (sec)
    float solution_age;  // solution age (sec)
    uint8_t num_sats_tracked;  // number of satellites tracked
    uint8_t num_sats_in_solution;  // number of satellites used in solution
    uint8_t num_sats_l1;  // number of L1/E1/B1 satellites used in solution
    uint8_t num_sats_multi;  // number of multi-frequency satellites used in solution
    uint8_t reserved;  // reserved
    uint8_t extended_solution_status;  // extended solution status - OEMV and greater only
    uint8_t galileo_beidou_used_mask;
    uint8_t gps_glonass_used_mask;
};
static_assert(sizeof(BestPos) == 72, "Incorrect size of BestPos");

struct BestVel {
    SolutionStatus solution_status;  // Solution status
    SolutionType velocity_type;
    float latency;  // measure of the latency of the velocity time tag in seconds
    float age;  // differential age in seconds
    double horizontal_speed;  // horizontal speed in m/s
    double track_over_ground;  // direction of travel in degrees
    double vertical_speed;  // vertical speed in m/s
    float reserved;
};
static_assert(sizeof(BestVel) == 44, "Incorrect size of BestVel");

// IMU data corrected for gravity, the earth’s rotation and estimated sensor errors.
struct CorrImuData {
    uint32_t gps_week;
    double gps_seconds;  // seconds of week
    // All the measurements are in the SPAN computational frame: right, forward, up.
    double x_angle_change;  // change in angle around x axis in radians
    double y_angle_change;  // change in angle around y axis in radians
    double z_angle_change;  // change in angle around z axis in radians
    double x_velocity_change;  // change in velocity along x axis in m/s
    double y_velocity_change;  // change in velocity along y axis in m/s
    double z_velocity_change;  // change in velocity along z axis in m/s
};
static_assert(sizeof(CorrImuData) == 60, "Incorrect size of CorrImuData");

struct InsCov {
    uint32_t gps_week;
    double gps_seconds;  // seconds of week
    double position_covariance[9];  // Position covariance matrix [m^2] (xx,xy,xz,yz,yy,...)
    double attitude_covariance[9];  // Attitude covariance matrix [deg^2] (xx,xy,xz,yz,yy,...)
    double velocity_covariance[9];  // Velocity covariance matrix [(m/s)^2] (xx,xy,xz,yz,yy,...)
};
static_assert(sizeof(InsCov) == 228, "Incorrect size of InsCov");

enum class InsStatus : uint32_t {
    INACTIVE = 0,
    ALIGNING,
    HIGH_VARIANCE,
    SOLUTION_GOOD,
    SOLUTION_FREE = 6,
    ALIGNMENT_COMPLETE,
    DETERMINING_ORIENTATION,
    WAITING_INITIAL_POS,
};

struct InsPva {
    uint32_t gps_week;
    double gps_seconds;  // seconds of week
    double latitude;  // in degrees
    double longitude;  // in degrees
    double height;  // Ellipsoidal height - WGS84 (m)
    double north_velocity;  // velocity in a northerly direction (m/s)
    double east_velocity;  // velocity in an easterly direction (m/s)
    double up_velocity;  // velocity in an up direction
    double roll;  // right handed rotation around y-axis (degrees)
    double pitch;  // right handed rotation around x-axis (degrees)
    double azimuth;  // left handed rotation around z-axis (degrees)
    InsStatus status;  // status of the INS system
};
static_assert(sizeof(InsPva) == 88, "Incorrect size of InsPva");

enum class ImuType : uint8_t {
    // We currently use the following IMUs. We'll extend this list when a new IMU is introduced.
    IMAR_FSAS = 13,  // iMAR iIMU-FSAS
    ISA100C = 26,  // Northrop Grumman Litef ISA-100C
    ADIS16488 = 31,  // Analog Devices ADIS16488
    STIM300 = 32,  // Sensonor STIM300
    ISA100 = 34,  // Northrop Grumman Litef ISA-100
    ISA100_400HZ = 38,  // Northrop Grumman Litef ISA-100
    ISA100C_400HZ = 39,  // Northrop Grumman Litef ISA-100
};

struct RawImuX {
    uint8_t imu_error;  // Simple IMU error flag. 0 means IMU okay.
    ImuType imu_type;
    uint16_t gps_week;
    double gps_seconds;  // Seconds of week.
    uint32_t imuStatus;  // Status of the IMU. The content varies with IMU type.
    // All the measurements are in the IMU reference frame. Scale factors varies with IMU type.
    int32_t z_velocity_change;  // change in velocity along z axis.
    int32_t y_velocity_change_neg;  // -change in velocity along y axis.
    int32_t x_velocity_change;  // change in velocity along x axis.
    int32_t z_angle_change;  // change in angle around z axis.
    int32_t y_angle_change_neg;  // -change in angle around y axis.
    int32_t x_angle_change;  // change in angle around x axis.
};
static_assert(sizeof(RawImuX) == 40, "Incorrect size of RawImuX");

struct RawImu {
    uint32_t gps_week;
    double gps_seconds;  // Seconds of week.
    char imuStatus[4];  // Status of the IMU. The content varies with IMU type.
    int32_t z_velocity_change;  // change in velocity along z axis.
    int32_t y_velocity_change_neg;  // -change in velocity along y axis.
    int32_t x_velocity_change;  // change in velocity along x axis.
    int32_t z_angle_change;  // change in angle around z axis.
    int32_t y_angle_change_neg;  // -change in angle around y axis.
    int32_t x_angle_change;  // change in angle around x axis.
};
static_assert(sizeof(RawImu) == 40, "Incorrect size of RawImu");

#pragma pack(pop) // Back to whatever the previous packing mode was.

struct ImuParameter {
    double gyro_scale;
    double accel_scale;
    double sampling_rate_hz;
};

inline ImuParameter get_imu_parameter(ImuType type) {
    switch (type) {
    case ImuType::IMAR_FSAS:
        // 0.1 * (2 ** -8) * (math.pi / 180 / 3600), (0.05 * (2 ** -15)
        return {1.893803441835e-9, 1.52587890625e-6, 200.0};

    case ImuType::ADIS16488:
        // 720/2**31 deg/LSB, 200/2**31 m/s/LSB
        return {5.8516723170686385e-09, 9.31322574615478515625e-8, 200.0};

    case ImuType::STIM300:
        // 2**-21 deg/LSB, 2**-22 m/s/LSB
        return {8.32237840649762e-09, 2.384185791015625e-07, 125.0};

    case ImuType::ISA100:
    case ImuType::ISA100C:
        // 1.0e-9 rad/LSB, 2.0e-8 m/s/LSB
        return {1.0e-9, 2.0e-8, 200.0};

    case ImuType::ISA100_400HZ:
    case ImuType::ISA100C_400HZ:
        return {1.0e-9, 2.0e-8, 400.0};

    default:
        return {0.0, 0.0, 0.0};
    }
}

}  // namespace novatel
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

#endif  // DRIVERS_GNSS_NOVATEL_MESSAGES_H
