load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
        "-O2",
        "-DMODULE_NAME=\\\"localization\\\"",
        "-DENABLE_DEBUG_LIB",
        "-DENABLE_OPENCV",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/proto:geometry_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/drivers/gnss/proto:config_proto",
        "//modules/drivers/gnss/proto:gnss_best_pose_proto",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss/proto:imu_proto",
        "//modules/localization/common:localization_common",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_integ:localization_msf_local_integ",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map:localization_pyramid_map_pyramid_map",
        "//modules/localization/proto:gnss_pnt_result_proto",
        "//modules/localization/proto:localization_config_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:localization_status_proto",
        "//modules/localization/proto:measure_proto",
        "//modules/localization/proto:sins_pva_proto",
        "//modules/transform:transform_broadcaster_lib",
        "//third_party:boost",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_binary(
    name = "libmsf_localization_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":msf_localization_component_lib"],
)

cc_test(
    name = "msf_localization_test",
    size = "small",
    timeout = "short",
    srcs = ["msf_localization_test.cc"],
    copts = [
        "-Dprivate=public",
    ],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//cyber",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/localization/msf:msf_localization_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
