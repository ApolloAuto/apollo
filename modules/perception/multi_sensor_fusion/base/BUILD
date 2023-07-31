load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    deps = [
        ":base_forward_declaration",
        ":fusion_log",
        ":scene",
        ":sensor",
        ":sensor_data_manager",
        ":track",
        ":track_pool_types",
    ],
)

cc_library(
    name = "base_forward_declaration",
    hdrs = ["base_forward_declaration.h"],
)

cc_library(
    name = "fusion_log",
    hdrs = ["fusion_log.h"],
)

cc_library(
  name = "fusion_frame",
  hdrs = [
    "fusion_frame.h",
  ],
  deps = [
    ":scene",
    ":sensor",
    "//modules/perception/common/base:frame",
    "//modules/perception/common/base:object",
  ],
)

cc_library(
    name = "scene",
    srcs = ["scene.cc"],
    hdrs = ["scene.h"],
    deps = [
        ":track",
    ],
)

cc_library(
    name = "sensor",
    srcs = [
        "sensor.cc",
        "sensor_frame.cc",
        "sensor_object.cc",
    ],
    hdrs = [
        "sensor.h",
        "sensor_frame.h",
        "sensor_object.h",
    ],
    deps = [
        ":base_forward_declaration",
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/common/algorithm/sensor_manager",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_test(
    name = "sensor_frame_test",
    size = "small",
    srcs = ["sensor_frame_test.cc"],
    deps = [
        ":sensor",
        "@com_google_googletest//:gtest_main",
    ],
)

# todo(zero): need fix
# cc_test(
#     name = "sensor_test",
#     size = "small",
#     srcs = ["sensor_test.cc"],
#     deps = [
#         ":sensor",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

# todo(zero): need fix
# cc_test(
#     name = "sensor_object_test",
#     size = "small",
#     srcs = ["sensor_object_test.cc"],
#     deps = [
#         ":sensor",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cc_library(
    name = "sensor_data_manager",
    srcs = ["sensor_data_manager.cc"],
    hdrs = ["sensor_data_manager.h"],
    deps = [
        ":sensor",
        "//modules/perception/common/base:frame",
        "//modules/perception/common/algorithm/sensor_manager",
        "//modules/perception/common/lib/thread",
        "@com_google_googletest//:gtest",
    ],
)

# cc_test(
#     name = "sensor_data_manager_test",
#     size = "small",
#     srcs = ["sensor_data_manager_test.cc"],
#     data = [
#         "//modules/perception/data:perception_testdata",
#     ],
#     deps = [
#         ":sensor_data_manager",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cc_library(
    name = "track",
    srcs = ["track.cc"],
    hdrs = ["track.h"],
    deps = [
        ":sensor",
        "//cyber",
        "@com_google_googletest//:gtest",
    ],
)

# todo(zero): need fix
# cc_test(
#     name = "track_test",
#     size = "small",
#     srcs = ["track_test.cc"],
#     deps = [
#         ":track",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cc_library(
    name = "track_pool_types",
    srcs = ["track_pool_types.cc"],
    hdrs = ["track_pool_types.h"],
    deps = [
        ":track",
        "//modules/perception/common/base:object_pool",
    ],
)

apollo_package()
cpplint()
