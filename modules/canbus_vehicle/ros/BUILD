load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_ros",
    srcs = [
        "ros_controller.cc",
        "ros_message_manager.cc",
        "ros_vehicle_factory.cc",
        "protocol/twist_fb.cc",
        "protocol/twist_cmd.cc",
        "protocol/ang_vel_fb.cc",
        "protocol/acc_fb.cc",
    ],
    hdrs = [
        "ros_controller.h",
        "ros_message_manager.h",
        "ros_vehicle_factory.h",
        "protocol/twist_fb.h",
        "protocol/twist_cmd.h",
        "protocol/ang_vel_fb.h",
        "protocol/acc_fb.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus_vehicle/ros/proto:ros_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)

apollo_component(
    name = "libros_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_ros"],
)

apollo_cc_test(
    name = "ros_vehicle_factory_test",
    size = "small",
    srcs = ["ros_vehicle_factory_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_canbus_vehicle_ros",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ros_message_manager_test",
    size = "small",
    srcs = ["ros_message_manager_test.cc"],
    deps = [
        ":apollo_canbus_vehicle_ros",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ros_controller_test",
    size = "small",
    srcs = ["ros_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":apollo_canbus_vehicle_ros",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
