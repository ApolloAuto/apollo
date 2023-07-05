# How to add a new GPS Receiver

## Introduction
GPS receiver is a device that receives information from GPS satellites and then calculates the device's geographical position, velocity and precise time. The device usually includes a receiver, an IMU (depends on the model), an interface to a wheel encoder, and a fusion engine that combines information from those sensors. The Default GPS receiver used in Apollo is Novatel cards. The instruction demonstrates how to add and use a new GPS Receiver.

## Steps to add a new GPS Receiver
Please follow the steps below to add a new GPS Receiver.
  1. Implement the new data parser for the new GPS receiver, by inheriting class `Parser`
  2. Add new interfaces in `Parser` class for the new GPS receiver
  3. In `config.proto`, add the new data format for the new GPS receiver
  4. In function `create_parser` from file data_parser.cpp, add the new parser instance for the new GPS receiver

Let's look at how to add the GPS Receiver using the above-mentioned steps for Receiver: `u-blox`.

### Step 1

Let us implement the new data parser for the new GPS receiver, by inheriting class `Parser`:

```cpp
class UbloxParser : public Parser {
public:
    UbloxParser();

    virtual MessageType get_message(MessagePtr& message_ptr);

private:
    bool verify_checksum();

    Parser::MessageType prepare_message(MessagePtr& message_ptr);

    // The handle_xxx functions return whether a message is ready.
    bool handle_esf_raw(const ublox::EsfRaw* raw, size_t data_size);
    bool handle_esf_ins(const ublox::EsfIns* ins);
    bool handle_hnr_pvt(const ublox::HnrPvt* pvt);
    bool handle_nav_att(const ublox::NavAtt *att);
    bool handle_nav_pvt(const ublox::NavPvt* pvt);
    bool handle_nav_cov(const ublox::NavCov *cov);
    bool handle_rxm_rawx(const ublox::RxmRawx *raw);

    double _gps_seconds_base = -1.0;

    double _gyro_scale = 0.0;

    double _accel_scale = 0.0;

    float _imu_measurement_span = 0.0;

    int _imu_frame_mapping = 5;

    double _imu_measurement_time_previous = -1.0;

    std::vector<uint8_t> _buffer;

    size_t _total_length = 0;

    ::apollo::drivers::gnss::Gnss _gnss;
    ::apollo::drivers::gnss::Imu _imu;
    ::apollo::drivers::gnss::Ins _ins;
};

```

### Step 2

Let us now add the new interfaces in the Parser class for the new GPS receiver:

Add the function `create_ublox` in `Parser` class:

```cpp
class Parser {
public:
    // Return a pointer to a NovAtel parser. The caller should take ownership.
    static Parser* create_novatel();

    // Return a pointer to a u-blox parser. The caller should take ownership.
    static Parser* create_ublox();

    virtual ~Parser() {}

    // Updates the parser with new data. The caller must keep the data valid until get_message()
    // returns NONE.
    void update(const uint8_t* data, size_t length) {
        _data = data;
        _data_end = data + length;
    }

    void update(const std::string& data) {
        update(reinterpret_cast<const uint8_t*>(data.data()), data.size());
    }

    enum class MessageType {
        NONE,
        GNSS,
        GNSS_RANGE,
        IMU,
        INS,
        WHEEL,
        EPHEMERIDES,
        OBSERVATION,
        GPGGA,
    };

    // Gets a parsed protobuf message. The caller must consume the message before calling another
    // get_message() or update();
    virtual MessageType get_message(MessagePtr& message_ptr) = 0;

protected:
    Parser() {}

    // Point to the beginning and end of data. Do not take ownership.
    const uint8_t* _data = nullptr;
    const uint8_t* _data_end = nullptr;

private:
    DISABLE_COPY_AND_ASSIGN(Parser);
};

Parser* Parser::create_ublox() {
    return new UbloxParser();
}
```

### Step 3

In config.proto, let us add the new data format definition for the new GPS receiver:

Add `UBLOX_TEXT` and `UBLOX_BINARY` in the config file: modules/drivers/gnss/proto/config.proto

```txt
message Stream {
    enum Format {
        UNKNOWN = 0;
        NMEA = 1;
        RTCM_V2 = 2;
        RTCM_V3 = 3;

        NOVATEL_TEXT = 10;
        NOVATEL_BINARY = 11;

        UBLOX_TEXT = 20;
        UBLOX_BINARY = 21;
    }
... ...
```

### Step 4

In function `create_parser` from file data_parser.cpp, let us add the new parser instance for the new GPS receiver.
We will do so by adding code to process `config::Stream::UBLOX_BINARY` as below:

``` cpp
Parser* create_parser(config::Stream::Format format, bool is_base_station = false) {
    switch (format) {
    case config::Stream::NOVATEL_BINARY:
        return Parser::create_novatel();

    case config::Stream::UBLOX_BINARY:
        return Parser::create_ubloxl();

    default:
        return nullptr;
    }
}

```
