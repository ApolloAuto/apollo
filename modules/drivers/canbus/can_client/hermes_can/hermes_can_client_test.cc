// Copyright 2017, Baidu Inc. All rights reserved.

#include "modules/drivers/canbus/can_client/hermes_can/hermes_can_client.h"
#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

TEST(HermesCanClient, init) {
    CANCardParameter param;
    param.set_brand(CANCardParameter::HERMES_CAN);
    param.set_channel_id(CANCardParameter::CHANNEL_ID_ZERO);
    HermesCanClient hermes_can;
    EXPECT_TRUE(hermes_can.Init(param));
//    EXPECT_EQ(hermes_can.Start(), ErrorCode::CAN_CLIENT_ERROR_BASE);
//      EXPECT_EQ(hermes_can.Start(), ErrorCode::OK);

}

TEST(HermesCanClient, send) {
    CANCardParameter param;
    param.set_brand(CANCardParameter::HERMES_CAN);
    param.set_channel_id(CANCardParameter::CHANNEL_ID_ZERO);
    HermesCanClient hermes_can;
    EXPECT_TRUE(hermes_can.Init(param));
   
 
    //CanFrame can_frame[1];
    std::vector<CanFrame> frames;
    int32_t num = 0;

    
    CanFrame frame;
    frame.id = 0x60;
    frame.len = 8;
    frame.data[0] = 0;
    EXPECT_EQ(hermes_can.Send(frames, &num), ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED);

    frames.push_back(frame);
    num = 1;
    EXPECT_EQ(hermes_can.Start(), ErrorCode::OK);
    EXPECT_EQ(hermes_can.Send(frames, &num),ErrorCode::OK);
    frames.clear();
}

TEST(HermesCanClient, receiver) {
    CANCardParameter param;
    param.set_brand(CANCardParameter::HERMES_CAN);
    param.set_channel_id(CANCardParameter::CHANNEL_ID_ZERO);
    HermesCanClient hermes_can;
    EXPECT_TRUE(hermes_can.Init(param));
    
    std::vector<CanFrame> frames;
    int32_t num = 0;
    CanFrame frame;
    //frame.id = 0x60;
    //frame.len = 8;
    //frame.data[0] = 0;
    //frames.push_back(frame);
    //num = 1;
    EXPECT_EQ(hermes_can.Start(), ErrorCode::OK);
    EXPECT_EQ(hermes_can.Receive(&frames, &num), ErrorCode::OK);
}

} // namespace canbus
} // namespace idl_car
} // namespace baidu
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
