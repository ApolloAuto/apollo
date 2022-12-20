# 如何添加新的CAN卡

## 简介
控制器区域网络（CAN）是在许多微控制器和设备中密集使用的网络，用于在没有主计算机帮助的情况下在设备之间传输数据。

Apollo中使用的默认CAN卡是 **ESD CAN-PCIe卡**。您可以使用以下步骤添加新的CAN卡：

## 添加新CAN卡
添加新的CAN卡需要完成以下几个步骤:

1. 实现新CAN卡的`CanClient`类。
2. 在`CanClientFactory`中注册新的CAN卡。
3. 更新配置文件。

以下步骤展示了如何添加新的CAN卡 - 示例添加CAN卡到您的工程。

### 步骤 1

实现新CAN卡的CanClient类
下面的代码展示了如何实现 `CANClient` 类:

```cpp
#include <string>
#include <vector>

#include "hermes_can/include/bcan.h"
#include "modules/canbus/can_client/can_client.h"
#include "modules/canbus/common/canbus_consts.h"
#include "modules/common/proto/error_code.pb.h"

/**
 * @namespace apollo::canbus::can
 * @brief apollo::canbus::can
 */
namespace apollo {
namespace canbus {
namespace can {

/**
 * @class ExampleCanClient
 * @brief The class which defines a Example CAN client which inherits CanClient.
 */
class ExampleCanClient : public CanClient {
 public:
  /**
   * @brief Initialize the Example CAN client by specified CAN card parameters.
   * @param parameter CAN card parameters to initialize the CAN client.
   * @return If the initialization is successful.
   */
  bool Init(const CANCardParameter& parameter) override;

  /**
   * @brief Destructor
   */
  virtual ~ExampleCanClient() = default;

  /**
   * @brief Start the Example CAN client.
   * @return The status of the start action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Start() override;

  /**
   * @brief Stop the Example CAN client.
   */
  void Stop() override;

  /**
   * @brief Send messages
   * @param frames The messages to send.
   * @param frame_num The amount of messages to send.
   * @return The status of the sending action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Send(const std::vector<CanFrame>& frames,
                                 int32_t* const frame_num) override;

  /**
   * @brief Receive messages
   * @param frames The messages to receive.
   * @param frame_num The amount of messages to receive.
   * @return The status of the receiving action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Receive(std::vector<CanFrame>* const frames,
                                    int32_t* const frame_num) override;

  /**
   * @brief Get the error string.
   * @param status The status to get the error string.
   */
  std::string GetErrorString(const int32_t status) override;

 private:
  ...
  ...
};

}  // namespace can
}  // namespace canbus
}  // namespace apollo
```

### 步骤 2
在CanClientFactory中注册新CAN卡，
在 `CanClientFactory`中添加如下代码:
```cpp
void CanClientFactory::RegisterCanClients() {  
  Register(CANCardParameter::ESD_CAN, 
           []() -> CanClient* { return new can::EsdCanClient(); });  
  
  // register the new CAN card here.  
  Register(CANCardParameter::EXAMPLE_CAN,  
           []() -> CanClient* { return new can::ExampleCanClient(); });  
}  
```

### 步骤 3

接下来，需要更新配置文件
在`/modules/canbus/proto/can_card_parameter.proto`添加 EXAMPLE_CAN 

```proto
message CANCardParameter {
  enum CANCardBrand {
    FAKE_CAN = 0;
    ESD_CAN = 1;
    EXAMPLE_CAN = 2; // add new CAN card here.
  }
  ... ... 
}
```
Update `/modules/canbus/conf/canbus_conf.pb.txt`

```txt
... ... 
can_card_parameter {
  brand:EXAMPLE_CAN
  type: PCI_CARD // suppose the new can card is PCI_CARD
  channel_id: CHANNEL_ID_ZERO // suppose the new can card has CHANNEL_ID_ZERO
}
... ...
```
