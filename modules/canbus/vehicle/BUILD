package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_controller_base",
    srcs = ["vehicle_controller.cc"],
    hdrs = [
        "vehicle_controller.h",
    ],
    deps = [
        "//modules/canbus/can_client",
        "//modules/canbus/can_comm:can_sender",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:message_manager_base",
        "//modules/common/proto:error_code_proto",
        "//modules/control/proto:control_proto",
    ],
)

cc_library(
    name = "message_manager_base",
    srcs = [
        "message_manager.cc",
        "protocol_data.cc",
    ],
    hdrs = [
        "message_manager.h",
        "protocol_data.h",
    ],
    deps = [
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/proto:error_code_proto",
        "//modules/common/time",
        "//modules/control/proto:control_proto",
    ],
)

cc_library(
    name = "abstract_vehicle_factory",
    srcs = [
        "abstract_vehicle_factory.cc",
    ],
    hdrs = [
        "abstract_vehicle_factory.h",
    ],
    deps = [
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
    ],
)

cc_library(
    name = "lincoln_vehicle_factory",
    srcs = [
        "lincoln_vehicle_factory.cc",
    ],
    hdrs = [
        "lincoln_vehicle_factory.h",
        "vehicle_factory.h",
    ],
    deps = [
        ":abstract_vehicle_factory",
        "//modules/canbus/vehicle/lincoln:lincoln_controller",
        "//modules/canbus/vehicle/lincoln:lincoln_message_manager",
    ],
)

cc_library(
    name = "vehicle_factory",
    srcs = [
        "vehicle_factory.cc",
    ],
    hdrs = [
        "vehicle_factory.h",
    ],
    deps = [
        ":lincoln_vehicle_factory",
        "//modules/common/util:factory",
    ],
)

cc_test(
    name = "protocol_data_test",
    size = "small",
    srcs = [
        "protocol_data_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle:message_manager_base",
        "@gtest//:main",
    ],
)

cc_test(
    name = "lincoln_vehicle_factory_test",
    size = "small",
    srcs = ["lincoln_vehicle_factory_test.cc"],
    deps = [
        "//modules/canbus/vehicle:lincoln_vehicle_factory",
        "@gtest//:main",
    ],
)

cc_test(
    name = "message_manager_test",
    size = "small",
    srcs = ["message_manager_test.cc"],
    deps = [
        "//modules/canbus/vehicle:message_manager_base",
        "@gtest//:main",
    ],
)
