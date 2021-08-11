load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "vehicle_controller_base",
    srcs = ["vehicle_controller.cc"],
    hdrs = ["vehicle_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/proto:error_code_cc_proto",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "abstract_vehicle_factory",
    srcs = ["abstract_vehicle_factory.cc"],
    hdrs = ["abstract_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "vehicle_factory",
    srcs = ["vehicle_factory.cc"],
    hdrs = ["vehicle_factory.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":abstract_vehicle_factory",
        "//modules/canbus/vehicle/ch:ch_vehicle_factory",
        "//modules/canbus/vehicle/devkit:devkit_vehicle_factory",
        "//modules/canbus/vehicle/ge3:ge3_vehicle_factory",
        "//modules/canbus/vehicle/gem:gem_vehicle_factory",
        "//modules/canbus/vehicle/lexus:lexus_vehicle_factory",
        "//modules/canbus/vehicle/lincoln:lincoln_vehicle_factory",
        "//modules/canbus/vehicle/neolix_edu:neolix_edu_vehicle_factory",
        "//modules/canbus/vehicle/transit:transit_vehicle_factory",
        "//modules/canbus/vehicle/wey:wey_vehicle_factory",
        "//modules/canbus/vehicle/zhongyun:zhongyun_vehicle_factory",
        "//modules/common/util:factory",
    ],
)

cc_test(
    name = "vehicle_factory_test",
    size = "small",
    srcs = ["vehicle_factory_test.cc"],
    deps = [
        ":vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
