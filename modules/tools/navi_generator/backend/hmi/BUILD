load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "topics_service",
    srcs = [
        "topics_service.cc",
    ],
    hdrs = [
        "topics_service.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math:quaternion",
        "//modules/common/monitor_log",
        "//modules/common/proto:common_proto",
        "//modules/common/util",
        "//modules/localization/proto:localization_proto",
        "//modules/tools/navi_generator/backend/common:navi_generator_gflags",
        "//modules/tools/navi_generator/backend/webserver:navi_generator_websocket",
        "//third_party/json",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "topics_updater",
    srcs = [
        "topics_updater.cc",
    ],
    hdrs = [
        "topics_updater.h",
    ],
    linkopts = [
        "-lboost_thread",
    ],
    deps = [
        ":topics_service",
        "//modules/common/util:json_util",
        "//modules/common/util:map_util",
        "//modules/common/util:util",
        "//modules/tools/navi_generator/backend/common:navi_generator_gflags",
        "//modules/tools/navi_generator/backend/webserver:navi_generator_websocket",
        "@com_google_protobuf//:protobuf",
    ],
)

cpplint()
