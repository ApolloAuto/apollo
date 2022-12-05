load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "testdata",
    srcs = glob(["local_map/test_data/**"])
)

cc_library(
    name = "msf_localization_component_lib",
    srcs = [
        "msf_localization.cc",
        "msf_localization_component.cc",
    ],
    hdrs = [
        "msf_localization.h",
        "msf_localization_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"localization\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_cc_proto",
        "//modules/common_msgs/sensor_msgs:heading_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
        "//modules/localization/common:localization_gflags",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_integ",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map",
        "//modules/localization/proto:gnss_pnt_result_cc_proto",
        "//modules/common_msgs/localization_msgs:gps_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/localization/proto:localization_config_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_status_cc_proto",
        "//modules/localization/proto:measure_cc_proto",
        "//modules/localization/proto:sins_pva_cc_proto",
        "//modules/transform:transform_broadcaster",
        "@boost",
        "@com_github_google_glog//:glog",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libmsf_localization_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":msf_localization_component_lib"],
)

cc_test(
    name = "msf_localization_test",
    size = "enormous",
    srcs = ["msf_localization_test.cc"],
    copts = [
        "-fno-access-control",
    ],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/localization/msf:msf_localization_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

install(
    name = "install",
    library_dest = "localization/lib/msf",
    data = [":testdata"],
    data_dest = "localization/addition_data/msf",
    targets = [
        ":libmsf_localization_component.so",
    ],
    deps = [
        "//modules/localization/msf/local_tool/map_creation:install",
        "//modules/localization/msf/local_tool/data_extraction:install",
        "//modules/localization/msf/local_tool/local_visualization/offline_visual:install",
        "//modules/localization/msf/local_tool/local_visualization/online_visual:install",
        #"//modules/localization/msf/local_tool/ndt_mapping:install",
        "//modules/localization/msf/params:install",
    ],
)

cpplint()
