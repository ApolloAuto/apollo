load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "ndt_lidar_locator",
    srcs = ["lidar_locator_ndt.cc"],
    hdrs = [
        "lidar_locator_ndt.h",
        "ndt_solver.h",
        "ndt_solver.hpp",
        "ndt_voxel_grid_covariance.h",
        "ndt_voxel_grid_covariance.hpp",
    ],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/util:util_tool",
        "//modules/localization/common:localization_gflags",
        "//modules/localization/msf:apollo_localization_msf",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_github_google_glog//:glog",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob([
        "ndt/test_data/**",
    ]),
)

apollo_cc_test(
    name = "ndt_lidar_locator_test",
    size = "large",
    timeout = "short",
    srcs = ["lidar_locator_ndt_test.cc"],
    data = [":test_data"],
    deps = [
        ":ndt_lidar_locator",
        "//cyber",
        "//modules/localization/msf:apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ndt_solver_test",
    size = "large",
    timeout = "short",
    srcs = ["ndt_solver_test.cc"],
    data = [":test_data"],
    deps = [
        ":ndt_lidar_locator",
        "//cyber",
        "//modules/localization/msf:apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
