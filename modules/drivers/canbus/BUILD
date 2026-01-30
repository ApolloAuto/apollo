load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "copts_if_esd_can", "if_esd_can")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_drivers_canbus",
    srcs = [
        "sensor_gflags.cc",
        "can_client/can_client_factory.cc",
        "can_client/fake/fake_can_client.cc",
        "can_client/hermes_can/hermes_can_client.cc",
        "can_client/socket/socket_can_client_raw.cc",
        "common/byte.cc",
    ],
    hdrs = [
        "sensor_gflags.h",
        "can_client/can_client.h",
        "can_client/can_client_factory.h",
        "can_client/fake/fake_can_client.h",
        "can_client/hermes_can/bcan.h",
        "can_client/hermes_can/bcan_defs.h",
        "can_client/hermes_can/hermes_can_client.h",
        "can_client/socket/socket_can_client_raw.h",
        "can_comm/can_receiver.h",
        "can_comm/can_sender.h",
        "can_comm/message_manager.h",
        "can_comm/protocol_data.h",
        "common/byte.cc",
        "common/byte.h",
        "common/canbus_consts.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//cyber",
        "//modules/common/util:common_util",
        "//modules/common/monitor_log",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/drivers_msgs:can_card_parameter_cc_proto",
        "//third_party/can_card_library/hermes_can",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_googletest//:gtest",
    ],
)

apollo_cc_test(
    name = "fake_can_client_test",
    size = "small",
    srcs = ["can_client/fake/fake_can_client_test.cc"],
    deps = [
        "//cyber",
        ":apollo_drivers_canbus",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "hermes_can_client_test",
    size = "small",
    srcs = ["can_client/hermes_can/hermes_can_client_test.cc"],
    deps = [
        "//cyber",
        ":apollo_drivers_canbus",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "socket_can_client_raw_test",
    size = "small",
    srcs = ["can_client/socket/socket_can_client_raw_test.cc"],
    deps = [
        "//cyber",
        ":apollo_drivers_canbus",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "can_sender_test",
    size = "small",
    srcs = ["can_comm/can_sender_test.cc"],
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        ":apollo_drivers_canbus",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "can_receiver_test",
    size = "small",
    srcs = ["can_comm/can_receiver_test.cc"],
    linkstatic = True,
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:drive_state_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        ":apollo_drivers_canbus",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "protocol_data_test",
    size = "small",
    srcs = ["can_comm/protocol_data_test.cc"],
    linkstatic = True,
    deps = [
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        ":apollo_drivers_canbus",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "message_manager_test",
    size = "small",
    srcs = ["can_comm/message_manager_test.cc"],
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        ":apollo_drivers_canbus",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "esd_can_client_test",
    size = "small",
    srcs = if_esd_can(["can_client/esd/esd_can_client_test.cc"]),
    deps = if_esd_can([
        ":apollo_drivers_canbus",
        "//cyber",
    ]) + [
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_binary(
    name = "can_client_tool",
    srcs = ["can_client/can_client_tool.cc"],
    deps = [
        "//cyber",
        "//modules/common_msgs/drivers_msgs:can_card_parameter_cc_proto",
        ":apollo_drivers_canbus",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_test(
    name = "can_client_factory_test",
    size = "small",
    srcs = ["can_client/can_client_factory_test.cc"],
    deps = [
        "//modules/common_msgs/drivers_msgs:can_card_parameter_cc_proto",
        ":apollo_drivers_canbus",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
