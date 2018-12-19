load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ndt_lidar_locator",
    srcs = [
        "lidar_locator_ndt.cc",
    ],
    hdrs = [
        "lidar_locator_ndt.h",
        "ndt_solver.h",
        "ndt_solver.hpp",
        "ndt_voxel_grid_covariance.h",
        "ndt_voxel_grid_covariance.hpp",
    ],
    deps = [
        "//modules/common/time",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/util:file_util",
        "//modules/localization/common:localization_common",
        "//modules/localization/msf/local_map/ndt_map:localization_msf_ndt_map",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//cyber",
        "@glog//:glog",
        "@eigen//:eigen",
        "@pcl//:pcl",
        "@yaml_cpp//:yaml",
    ],
)

cpplint()
