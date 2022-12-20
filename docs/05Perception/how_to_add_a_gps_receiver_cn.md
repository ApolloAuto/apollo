# 如何添加新的GPS接收器

## 简介
GPS接收器是一种从GPS卫星上接收信息，然后根据这些信息计算设备地理位置、速度和精确时间的设备。这种设备通常包括一个接收器，一个IMU（Inertial measurement unit，惯性测量单元），一个针对轮编码器的接口以及一个将各传感器获取的数据融合到一起的融合引擎。Apollo系统中默认使用Novatel 板卡，该说明详细介绍如何添加并使用一个新的GPS接收器。

## 添加GPS新接收器的步骤
请按照下面的步骤添加新的GPS接收器.
  1. 通过继承基类“Parser”，实现新GPS接收器的数据解析器
  2. 在Parser类中为新GPS接收器添加新接口
  3. 在文件`config.proto`中, 为新GPS接收器添加新数据格式
  4. 在函数`create_parser`（见文件data_parser.cpp）, 为新GPS接收器添加新解析器实例

下面让我们用上面的方法来添加u-blox GPS接收器。

### 步骤 1

通过继承类“Parser”，为新GPS接收器实现新的数据解析器:

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

### 步骤 2

在Parser类中，为新GPS接收器添加新的接口:

在Parser类中添加函数‘create_ublox‘:

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

### 步骤 3

在config.proto文件中, 为新的GPS接收器添加新的数据格式定义:

在配置文件（modules/drivers/gnss/proto/config.proto）中添加`UBLOX_TEXT` and `UBLOX_BINARY` 

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

### 步骤 4

在函数`create_parser`（见data_parser.cpp）, 为新GPS接收器添加新解析器实例.
我们将通过添加处理`config::Stream::UBLOX_BINARY`的代码实现上面的步骤，具体如下。

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
