load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools/platform:build_defs.bzl", "if_aarch64")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_cc_library(
    name = "apollo_perception_common_algorithm",
    srcs = [
        "geometry/camera_homography.cc",
        "geometry/roi_filter.cc",
        "graph/connected_component_analysis.cc",
        "graph/disjoint_set.cc",
        "graph/graph_segmentor.cc",
        "i_lib/pc/i_ground.cc",
        "image_processing/hough_transfer.cc",
        "io/io_util.cc",
        "sensor_manager/sensor_manager.cc",
    ],
    hdrs = [
        "geometry/basic.h",
        "geometry/camera_homography.h",
        "geometry/common.h",
        "geometry/convex_hull_2d.h",
        "geometry/roi_filter.h",
        "graph/conditional_clustering.h",
        "graph/connected_component_analysis.h",
        "graph/disjoint_set.h",
        "graph/gated_hungarian_bigraph_matcher.h",
        "graph/graph_segmentor.h",
        "graph/hungarian_optimizer.h",
        "graph/secure_matrix.h",
        "i_lib/algorithm/i_sort.h",
        "i_lib/core/i_alloc.h",
        "i_lib/core/i_basic.h",
        "i_lib/core/i_blas.h",
        "i_lib/core/i_constant.h",
        "i_lib/core/i_rand.h",
        "i_lib/da/i_ransac.h",
        "i_lib/geometry/i_line.h",
        "i_lib/geometry/i_plane.h",
        "i_lib/geometry/i_util.h",
        "i_lib/pc/i_ground.h",
        "i_lib/pc/i_struct_s.h",
        "i_lib/pc/i_util.h",
        "image_processing/hough_transfer.h",
        "io/io_util.h",
        "point_cloud_processing/common.h",
        "point_cloud_processing/downsampling.h",
        "sensor_manager/sensor_manager.h",
    ] + if_aarch64(["i_lib/pc/sse2neon.h",]),
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/proto:sensor_meta_schema_cc_proto",
        "@boost",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "basic_test",
    size = "small",
    srcs = ["geometry/basic_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "camera_homography_test",
    size = "small",
    srcs = ["geometry/camera_homography_test.cc"],
    deps = [
        ":apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "roi_filter_test",
    size = "small",
    srcs = ["geometry/roi_filter_test.cc"],
    deps = [
        ":apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "convex_hull_2d_test",
    size = "small",
    srcs = ["geometry/convex_hull_2d_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "gated_hungarian_bigraph_matcher_test",
    size = "small",
    srcs = ["graph/gated_hungarian_bigraph_matcher_test.cc"],
    deps = [
        ":apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "conditional_clustering_test",
    size = "small",
    srcs = ["graph/conditional_clustering_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "connected_component_analysis_test",
    size = "small",
    srcs = ["graph/connected_component_analysis_test.cc"],
    deps = [
        ":apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "hungarian_optimizer_test",
    size = "small",
    srcs = ["graph/hungarian_optimizer_test.cc"],
    deps = [
        ":apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "secure_matrix_test",
    size = "small",
    srcs = ["graph/secure_matrix_test.cc"],
    deps = [
        ":apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "disjoint_set_test",
    size = "small",
    srcs = ["graph/disjoint_set_test.cc"],
    deps = [
        ":apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "graph_segmentor_test",
    size = "small",
    srcs = ["graph/graph_segmentor_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "hough_transfer_test",
    size = "small",
    srcs = ["image_processing/hough_transfer_test.cc"],
    deps = [
        ":apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

# apollo_cc_test(
#     name = "io_util_test",
#     size = "small",
#     srcs = ["io/io_util_test.cc"],
#     copts = ["-fno-access-control"],
#     data = [
#         "//modules/perception/data:perception_testdata",
#     ],
#     deps = [
#         ":apollo_perception_common_algorithm",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

apollo_package()

cpplint()