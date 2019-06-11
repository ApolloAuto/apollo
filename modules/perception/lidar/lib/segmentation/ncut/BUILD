load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ncut",
    srcs = [
        "ncut.cc",
    ],
    hdrs = [
        "ncut.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/lidar/lib/segmentation/ncut/common:flood_fill",
        "//modules/perception/lidar/lib/segmentation/ncut/common:lr_classifier",
        "//modules/perception/lidar/lib/segmentation/ncut/proto:ncut_param_proto",
    ],
)

cc_library(
    name = "ncutsegmentation",
    srcs = [
        "ncut_segmentation.cc",
    ],
    hdrs = [
        "ncut_segmentation.h",
    ],
    copts = [
        "-fopenmp",
    ],
    linkopts = [
        "-lgomp",
    ],
    deps = [
        ":ncut",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/lib/thread",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/segmentation/ncut/common:flood_fill",
        "//modules/perception/lidar/lib/segmentation/ncut/common:lr_classifier",
        "//modules/perception/lidar/lib/segmentation/ncut/proto:ncut_param_proto",
    ],
)

cpplint()
