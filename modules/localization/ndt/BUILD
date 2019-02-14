load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ndt_localization_component_lib",
    srcs = [
        "ndt_localization.cc",
        "localization_pose_buffer.cc",
        "ndt_localization_component.cc",
    ],
    hdrs = [
        "ndt_localization.h",
        "localization_pose_buffer.h",
        "ndt_localization_component.h",
    ],
    copts = ["-DMODULE_NAME=\\\"NDTlocalization\\\""],
    deps = [
        "//cyber",
        "//modules/common/time",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/util:message_util",
        "//modules/transform:tf2_buffer_lib",
        "//modules/common/proto:geometry_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/gnss/proto:ins_proto",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/transform:transform_broadcaster_lib",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/common:localization_common",
        "//modules/localization/ndt/ndt_locator:ndt_lidar_locator",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "@glog//:glog",
        "@eigen//:eigen",
        "@yaml_cpp//:yaml",
    ],
)

cc_binary(
    name = "libndt_localization_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":ndt_localization_component_lib"],
)

cc_test(
    name = "ndt_localization_test",
    size = "small",
    timeout = "short",
    srcs = [
        "ndt_localization_test.cc",
    ],
    deps = [
        "//cyber",
        "//modules/localization/ndt:ndt_localization_component_lib",
        "@gtest//:main",
    ],
)

cc_test(
    name = "ndt_localization_pose_buffer_test",
    size = "small",
    timeout = "short",
    srcs = [
        "localization_pose_buffer_test.cc"
    ],
    deps = [
        "//cyber",
        "//modules/localization/ndt:ndt_localization_component_lib",
        "@gtest//:main",
    ],
)

cpplint()
