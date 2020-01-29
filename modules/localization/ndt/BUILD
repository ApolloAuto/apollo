load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ndt_localization_component_lib",
    srcs = [
        "localization_pose_buffer.cc",
        "ndt_localization.cc",
        "ndt_localization_component.cc",
    ],
    hdrs = [
        "localization_pose_buffer.h",
        "ndt_localization.h",
        "ndt_localization_component.h",
    ],
    copts = ["-DMODULE_NAME=\\\"NDTlocalization\\\""],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/proto:geometry_proto",
        "//modules/common/time",
        "//modules/common/util:message_util",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss/proto:ins_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/common:localization_common",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/ndt/ndt_locator:ndt_lidar_locator",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/transform:tf2_buffer_lib",
        "//modules/transform:transform_broadcaster_lib",
        "//third_party:boost",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_glog//:glog",
        "@eigen",
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
    srcs = ["ndt_localization_test.cc"],
    deps = [
        "//cyber",
        "//modules/localization/ndt:ndt_localization_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ndt_localization_pose_buffer_test",
    size = "small",
    timeout = "short",
    srcs = ["localization_pose_buffer_test.cc"],
    deps = [
        "//cyber",
        "//modules/localization/ndt:ndt_localization_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
