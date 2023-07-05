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

// This defines enums and structures for parsing NovAtel binary messages. Please
// refer to NovAtel's
// documents for details about these messages.
//  http://www.novatel.com/assets/Documents/Manuals/om-20000129.pdf
//  http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf

#pragma once

#include <cstdint>
#include <limits>

#include "modules/drivers/gnss/proto/config.pb.h"

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
  RAWIMUS = 325,
  RAWIMUX = 1461,
  RAWIMUSX = 1462,
  MARK1PVA = 1067,
  GPGGA = 218,
  BDSEPHEMERIS = 1696,
  GLOEPHEMERIS = 723,
  GPSEPHEMERIS = 7,
  RANGE = 43,
  HEADING = 971,
  IMURATECORRIMUS = 1362,
};

// Every binary message has 32-bit CRC performed on all data including the
// header.
constexpr uint16_t CRC_LENGTH = 4;

#pragma pack(push, 1)  // Turn off struct padding.

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
  uint16_t
      message_length;  // Message length (not including the header nor the CRC).
  uint16_t sequence;   // Counts down from N-1 to 0 for multiple related logs.
  uint8_t idle;  // Time the processor was idle in last second between logs with
                 // same ID.
  uint8_t time_status;     // Indicates the quality of the GPS time.
  uint16_t gps_week;       // GPS Week number.
  uint32_t gps_millisecs;  // Milliseconds of week.
  uint32_t status;         // Receiver status.
  uint16_t reserved;
  uint16_t version;  // Receiver software build number.
};
static_assert(sizeof(LongHeader) == 28, "Incorrect size of LongHeader");

struct ShortHeader {
  SyncByte sync[3];
  uint8_t
      message_length;  // Message length (not including the header nor the CRC).
  MessageId message_id;
  uint16_t gps_week;       // GPS Week number.
  uint32_t gps_millisecs;  // Milliseconds of week.
};
static_assert(sizeof(ShortHeader) == 12, "Incorrect size of ShortHeader");

enum class SolutionStatus : uint32_t {
  SOL_COMPUTED = 0,  // solution computed
  INSUFFICIENT_OBS,  // insufficient observations
  NO_CONVERGENCE,    // no convergence
  SINGULARITY,       // singularity at parameters matrix
  COV_TRACE,         // covariance trace exceeds maximum (trace > 1000 m)
  TEST_DIST,   // test distance exceeded (max of 3 rejections if distance > 10
               // km)
  COLD_START,  // not yet converged from cold start
  V_H_LIMIT,   // height or velocity limits exceeded
  VARIANCE,    // variance exceeds limits
  RESIDUALS,   // residuals are too large
  INTEGRITY_WARNING = 13,  // large residuals make position questionable
  PENDING = 18,  // receiver computes its position and determines if the fixed
                 // position is valid
  INVALID_FIX = 19,   // the fixed position entered using the fix position
                      // command is invalid
  UNAUTHORIZED = 20,  // position type is unauthorized
  INVALID_RATE =
      22,  // selected logging rate is not supported for this solution type
  NONE = std::numeric_limits<uint32_t>::max(),
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
  RTK_DIRECT_INS = 51,  // RTK filter is directly initialized
                        // from the INS filter.
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

struct BDS_Ephemeris {
  uint32_t satellite_id;  // ID/ranging code
  uint32_t week;          // week number
  double ura;             // user range accuracy(meters).
                          // This is the evaluated URAI/URA lookup-table value
  uint32_t health1;       // Autonomous satellite health flag.
                          // 0 means broadcasting satellite is good, 1 means not
  double tdg1;  // Equipment group delay differential for the B1 signal(seconds)
  double tdg2;  // Equipment group delay differential for the B2 signal(seconds)
  uint32_t aodc;  // Age of data, clock
  uint32_t toc;   // Reference time of clock parameters
  double a0;      // Constant term of clock correction polynomial(seconds)
  double a1;  // Linear term of clock correction polynomial (seconds/seconds)
  double a2;  // Quadratic term of clock correction polynomial
              // (second/seconds^2)
  uint32_t aode;     // Age of data, ephemeris
  uint32_t toe;      // Reference time of ephemeris parameters
  double rootA;      // Square root of semi-major axis (sqrt(meters))
  double ecc;        // Eccentricity (sqrt(meters))
  double omega;      // Argument of perigee
  double delta_N;    // Mean motion difference from computed value
                     // (radians/second)
  double M0;         // Mean anomaly at reference time (radians)
  double omega0;     // Longitude of ascending node of orbital of plane computed
                     // according to reference time(radians)
  double rra;        // Rate of right ascension(radians/second)
  double inc_angle;  // inclination angle at reference time(radians)
  double idot;       // Rate of inclination angle(radians/second)
  double cuc;  // Amplitude of cosine harmonic correction term to the argument
               // of latitude(radians)
  double cus;  // Amplitude of sine harmonic correction term to the argument
               // of latitude(radians)
  double crc;  // Amplitude of cosine harmonic correction term to
               // the orbit radius(meters)
  double crs;  // Amplitude of sine harmonic correction term to
               // the orbit radius(meters)
  double cic;  // Amplitude of cosine harmonic correction term to the angle of
               // inclination(radians)
  double cis;  // Amplitude of sine harmonic correction term to the angle of
               // inclination(radians)
};
static_assert(sizeof(BDS_Ephemeris) == 196, "Incorrect size of BDS_Ephemeris.");

struct GLO_Ephemeris {
  uint16_t sloto;      // Slot information offset-PRNidentification(Slot+37).
  uint16_t freqo;      // frequency channel offset for satellite
                       // in the range 0 to 20
  uint8_t sat_type;    // Satellite type where(0=GLO_SAT, 1=GLO_SAT_M,
                       // 2=GLO_SAT_K)
  uint8_t reserved_1;  // Reserved
  uint16_t e_week;     // reference week of ephemeris(GPS reference time)
  uint32_t e_time;     // reference time of ephemeris(GPS reference time) in ms
  uint32_t t_offset;   // integer seconds between GPS and GLONASS time.
                       // A positive value implies GLONASS is ahead
                       // of GPS reference time.
  uint16_t Nt;  // Calendar number of day within 4 year interval starting at
                // Jan 1 of leap year
  uint8_t reserved_2;  // Reserved
  uint8_t reserved_3;  // Reserved
  uint32_t issue;      // 15 minute interval number corresponding to ephemeris
                       // reference time
  uint32_t health;     // Ephemeris health where 0-3=GOOD, 4-15=BAD
  double pos_x;  // X coordinate for satellite at reference time (PZ-90.02),
                 // in meters
  double pos_y;  // Y coordinate for satellite at reference time (PZ-90.02),
                 // in meters
  double pos_z;  // Z coordinate for satellite at reference time (PZ-90.02),
                 // in meters
  double vel_x;  // X coordinate for satellite velocity at reference
                 // time(PZ-90.02), in meters/s
  double vel_y;  // Y coordinate for satellite velocity at reference
                 // time(PZ-90.02), in meters/s
  double vel_z;  // Z coordinate for satellite velocity at reference
                 // time(PZ-90.02), in meters/s
  double acc_x;  // X coordinate for lunisolar acceleration at reference
                 // time(PZ-90.02), in meters/s/s
  double acc_y;  // Y coordinate for lunisolar acceleration at reference
                 // time(PZ-90.02), in meters/s/s
  double acc_z;  // Z coordinate for lunisolar acceleration at reference
                 // time(PZ-90.02), in meters/s/s
  double tau_n;  // Correction to the nth satellite time t_n relative to
                 // GLONASS time_t, in seconds
  double delta_tau_n;  // Time difference between navigation RF signal
                       // transmitted in L2 sub-band and
                       // navigation RF signal transmitted in
                       // L1 sub-band by nth satellite , in seconds
  double gamma;        // frequency correction , in seconds/second
  uint32_t Tk;   // Time of frame start(since start of GLONASS day), in seconds
  uint32_t P;    // technological parameter
  uint32_t Ft;   // User range
  uint32_t age;  // age of data, in days
  uint32_t Flags;  // information flags
};
static_assert(sizeof(GLO_Ephemeris) == 144, "Incorrect size of GLO_Ephemeris.");

struct GPS_Ephemeris {
  uint32_t prn;      // Satellite prn number
  double tow;        // Time stamp of subframe 0
  uint32_t health;   // Health status -a 6-bit health code as defined in
                     // ICD-GPS-200
  uint32_t iode1;    // issue of ephemeris data 1
  uint32_t iode2;    // issue of ephemeris data 2
  uint32_t week;     // GPS reference week number
  uint32_t z_week;   // Z count week number
  double toe;        // reference time for ephemeris, seconds
  double A;          // semi-major axis, metres
  double delta_A;    // Mean motion difference, radians/second
  double M_0;        // Mean anomaly of reference time, radians
  double ecc;        // Eccentricity. dimensionless-quantity defined for
                     // a conic section where e=0 is a circle, e=1 is a parabola
                     // 0<e<1 os an ellipse and e>1 is a hyperbola
  double omega;      // Argument of perigee, radians -measurement along the
                     // orbital path from the ascending node to the point where
                     // the SV os closest to the earth, in the direction of
                     // the SV's motion
  double cuc;        // Argument of latitude
  double cus;        // Argument of latitude
  double crc;        // Orbit radius
  double crs;        // Orbit radius
  double cic;        // Inclination
  double cis;        // Inclination
  double I_0;        // Inclination angle at reference time, radians
  double dot_I;      // Rate of inclination angle, radians/second
  double omega_0;    // right ascension, radians
  double dot_omega;  // rate of right ascension, radians/second
  uint32_t iodc;     // issue of data clock
  double toc;        // SV clock correction term, seconds
  double tgd;        // Estimated group delay difference seconds
  double af0;        // Clock aging parameter. seconds
  double af1;        // Clock aging parameter
  double af2;        // Clock aging parameter
  uint32_t AS;       // Anti-spoofing on : 0=false, 1=true
  double N;          // Corrected mean motion, radians/second
  double ura;        // User Range Acceracy variance.
};
static_assert(sizeof(GPS_Ephemeris) == 224, "Incorrect size of GPS_Ephemeris.");

struct BestPos {
  SolutionStatus solution_status;
  SolutionType position_type;
  double latitude;               // in degrees
  double longitude;              // in degrees
  double height_msl;             // height above mean sea level in meters
  float undulation;              // undulation = height_wgs84 - height_msl
  DatumId datum_id;              // datum id number
  float latitude_std_dev;        // latitude standard deviation (m)
  float longitude_std_dev;       // longitude standard deviation (m)
  float height_std_dev;          // height standard deviation (m)
  char base_station_id[4];       // base station id
  float differential_age;        // differential position age (sec)
  float solution_age;            // solution age (sec)
  uint8_t num_sats_tracked;      // number of satellites tracked
  uint8_t num_sats_in_solution;  // number of satellites used in solution
  uint8_t num_sats_l1;  // number of L1/E1/B1 satellites used in solution
  uint8_t
      num_sats_multi;  // number of multi-frequency satellites used in solution
  uint8_t reserved;    // reserved
  uint8_t extended_solution_status;  // extended solution status - OEMV and
                                     // greater only
  uint8_t galileo_beidou_used_mask;
  uint8_t gps_glonass_used_mask;
};
static_assert(sizeof(BestPos) == 72, "Incorrect size of BestPos");

struct BestVel {
  SolutionStatus solution_status;  // Solution status
  SolutionType velocity_type;
  float latency;  // measure of the latency of the velocity time tag in seconds
  float age;      // differential age in seconds
  double horizontal_speed;   // horizontal speed in m/s
  double track_over_ground;  // direction of travel in degrees
  double vertical_speed;     // vertical speed in m/s
  float reserved;
};
static_assert(sizeof(BestVel) == 44, "Incorrect size of BestVel");

// IMU data corrected for gravity, the earth's rotation and estimated sensor
// errors.
struct CorrImuData {
  uint32_t gps_week;
  double gps_seconds;  // seconds of week
  // All the measurements are in the SPAN computational frame: right, forward,
  // up.
  double x_angle_change;     // change in angle around x axis in radians
  double y_angle_change;     // change in angle around y axis in radians
  double z_angle_change;     // change in angle around z axis in radians
  double x_velocity_change;  // change in velocity along x axis in m/s
  double y_velocity_change;  // change in velocity along y axis in m/s
  double z_velocity_change;  // change in velocity along z axis in m/s
};
static_assert(sizeof(CorrImuData) == 60, "Incorrect size of CorrImuData");

struct InsCov {
  uint32_t gps_week;
  double gps_seconds;             // seconds of week
  double position_covariance[9];  // Position covariance matrix [m^2]
                                  // (xx,xy,xz,yz,yy,...)
  double attitude_covariance[9];  // Attitude covariance matrix [deg^2]
                                  // (xx,xy,xz,yz,yy,...)
  double velocity_covariance[9];  // Velocity covariance matrix [(m/s)^2]
                                  // (xx,xy,xz,yz,yy,...)
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
  NONE = std::numeric_limits<uint32_t>::max(),
};

struct InsPva {
  uint32_t gps_week;
  double gps_seconds;     // seconds of week
  double latitude;        // in degrees
  double longitude;       // in degrees
  double height;          // Ellipsoidal height - WGS84 (m)
  double north_velocity;  // velocity in a northerly direction (m/s)
  double east_velocity;   // velocity in an easterly direction (m/s)
  double up_velocity;     // velocity in an up direction
  double roll;            // right handed rotation around y-axis (degrees)
  double pitch;           // right handed rotation around x-axis (degrees)
  double azimuth;         // left handed rotation around z-axis (degrees)
  InsStatus status;       // status of the INS system
};
static_assert(sizeof(InsPva) == 88, "Incorrect size of InsPva");

struct InsPvaX {
  uint32_t ins_status;
  uint32_t pos_type;
  double latitude;   // in degrees
  double longitude;  // in degrees
  double height;     // Ellipsoidal height - WGS84 (m)
  float undulation;

  double north_velocity;  // velocity in a northerly direction (m/s)
  double east_velocity;   // velocity in an easterly direction (m/s)
  double up_velocity;     // velocity in an up direction
  double roll;            // right handed rotation around y-axis (degrees)
  double pitch;           // right handed rotation around x-axis (degrees)
  double azimuth;         // left handed rotation around z-axis (degrees)

  float latitude_std;
  float longitude_std;
  float height_std;
  float north_velocity_std;
  float east_velocity_std;
  float up_velocity_std;

  float roll_std;
  float pitch_std;
  float azimuth_std;
  uint32_t ext_slo_stat;
  uint16_t elapsed_time;
};
static_assert(sizeof(InsPvaX) == 126, "Incorrect size of InsPvaX");

// enum class ImuType : uint8_t {
//   // We currently use the following IMUs. We'll extend this list when a new
//   IMU
//   // is introduced.
//   IMAR_FSAS = 13,      // iMAR iIMU-FSAS
//   ISA100C = 26,        // Northrop Grumman Litef ISA-100C
//   ADIS16488 = 31,      // Analog Devices ADIS16488
//   STIM300 = 32,        // Sensonor STIM300
//   ISA100 = 34,         // Northrop Grumman Litef ISA-100
//   ISA100_400HZ = 38,   // Northrop Grumman Litef ISA-100
//   ISA100C_400HZ = 39,  // Northrop Grumman Litef ISA-100
//   G320N = 40,          // EPSON G320N
//   CPT_X25651 = 41,     // IMU@SPAN-CPT, and XingWangYuda 5651
//   BMI055 = 42          // BOSCH BMI055 IMU
// };

struct RawImuX {
  uint8_t imu_error;  // Simple IMU error flag. 0 means IMU okay.
  uint8_t imu_type;
  uint16_t gps_week;
  double gps_seconds;  // Seconds of week.
  uint32_t imuStatus;  // Status of the IMU. The content varies with IMU type.
  // All the measurements are in the IMU reference frame. Scale factors varies
  // with IMU type.
  int32_t z_velocity_change;      // change in velocity along z axis.
  int32_t y_velocity_change_neg;  // -change in velocity along y axis.
  int32_t x_velocity_change;      // change in velocity along x axis.
  int32_t z_angle_change;         // change in angle around z axis.
  int32_t y_angle_change_neg;     // -change in angle around y axis.
  int32_t x_angle_change;         // change in angle around x axis.
};
static_assert(sizeof(RawImuX) == 40, "Incorrect size of RawImuX");

struct RawImu {
  uint32_t gps_week;
  double gps_seconds;  // Seconds of week.
  uint32_t imuStatus;  // Status of the IMU. The content varies with IMU type.
  int32_t z_velocity_change;      // change in velocity along z axis.
  int32_t y_velocity_change_neg;  // -change in velocity along y axis.
  int32_t x_velocity_change;      // change in velocity along x axis.
  int32_t z_angle_change;         // change in angle around z axis.
  int32_t y_angle_change_neg;     // -change in angle around y axis.
  int32_t x_angle_change;         // change in angle around x axis.
};
static_assert(sizeof(RawImu) == 40, "Incorrect size of RawImu");

struct Heading {
  SolutionStatus solution_status;
  SolutionType position_type;
  float length;
  float heading;
  float pitch;
  float reserved;
  float heading_std_dev;
  float pitch_std_dev;
  char station_id[4];            // station id
  uint8_t num_sats_tracked;      // number of satellites tracked
  uint8_t num_sats_in_solution;  // number of satellites used in solution
  uint8_t num_sats_ele;
  uint8_t num_sats_l2;
  uint8_t solution_source;
  uint8_t extended_solution_status;
  uint8_t galileo_beidou_sig_mask;
  uint8_t gps_glonass_sig_mask;
};
static_assert(sizeof(Heading) == 44, "Incorrect size of Heading");

#pragma pack(pop)  // Back to whatever the previous packing mode was.

struct ImuParameter {
  double gyro_scale;
  double accel_scale;
  double sampling_rate_hz;
};

using ::apollo::drivers::gnss::config::ImuType;
inline ImuParameter GetImuParameter(ImuType type) {
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

    case ImuType::G320N:
      return {1.7044230976507124e-11, 2.3929443359375006e-10, 125.0};

    case ImuType::CPT_XW5651:
      return {1.0850694444444445e-07, 1.52587890625e-06, 100.0};

    case ImuType::UM442:
      return {6.6581059144655048e-6, 2.99127170628e-5, 20.0};

    case ImuType::IAM20680:
      // (1.0/65.5)/125.0 deg/LSB (1.0/8192.0)*9.80665/125.0 m/s/LSB
      return {0.0001221374045, 9.57680664e-06, 125};

    default:
      return {0.0, 0.0, 0.0};
  }
}

}  // namespace novatel
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
