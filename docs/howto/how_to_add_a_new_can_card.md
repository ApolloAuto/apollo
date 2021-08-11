# How to Add a New CAN Card

## Introduction
The Controller Area Network (CAN) is a network used intensively in many microcontrollers and devices to transfer data between devices without the assistance of a host computer.

The default CAN card used in Apollo is the **ESD CAN-PCIe card**. You can add a new CAN card using the steps below:

## Adding a New CAN Card
Complete the following required task sequence to add a new CAN card:

1. Implement the `CanClient` class of the new CAN card.
2. Register the new CAN card in `CanClientFactory`.
3. Update the config file.

The steps below show how you can add a new CAN card -- EXAMPLE CAN card to your stack.

### Step 1

Let us Implement the CanClient Class of the New CAN Card
Use the following code to implement the specific `CANClient` class:

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
 * @brief The class which defines an Example CAN client which inherits CanClient.
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

### Step 2
To register the New CAN Card in CanClientFactory,
add the following code to `CanClientFactory`:
```cpp
void CanClientFactory::RegisterCanClients() {
  Register(CANCardParameter::ESD_CAN,
           []() -> CanClient* { return new can::EsdCanClient(); });

  // register the new CAN card here.
  Register(CANCardParameter::EXAMPLE_CAN,
           []() -> CanClient* { return new can::ExampleCanClient(); });
}
```

### Step 3

Next, you would need to update the config File
Add the EXAMPLE_CAN into `/modules/drivers/canbus/proto/can_card_parameter.proto`

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

If you use radar, like Conti radar in apollo, its' canbus configuration file should alse be modified. Update `/modules/drivers/radar/conti_radar/conf/conti_radar_conf.pb.txt`

```txt
... ...
can_card_parameter {
  brand:EXAMPLE_CAN
  type: PCI_CARD // suppose the new can card is PCI_CARD
  channel_id: CHANNEL_ID_ZERO // suppose the new can card has CHANNEL_ID_ZERO
}
... ...
```
