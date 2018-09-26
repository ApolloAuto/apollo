load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# cc_library(
#     name = "cnn_segmentation",
#     srcs = [
#         "cnn_segmentation.cc",
#     ],
#     hdrs = [
#         "cnn_segmentation.h",
#     ],
#     deps = [
#         ":feature_generator",
#         "//framework:cybertron",
#     ],
# )

cc_library(
    name = "disjoint_set",
    hdrs = [
        "disjoint_set.h",
    ],
)

cc_library(
    name = "feature_generator",
    srcs = [
        "feature_generator.cc",
    ],
    hdrs = [
        "feature_generator.h",
    ],
    deps = [
        ":util",
        "//modules/perception/base",
        "//modules/perception/lidar/lib/segmentation/cnnseg/proto:cnnseg_param_proto",
        "@eigen",
    ],
)

cc_library(
    name = "util",
    hdrs = [
        "util.h",
    ],
)

cpplint()
