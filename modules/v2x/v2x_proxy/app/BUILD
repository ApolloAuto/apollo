load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "v2x_proxy_lib",
    srcs = [
        "v2x_proxy.cc",
        "main.cc",
    ],
    hdrs = [
        "v2x_proxy.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/adapters:adapter_gflags",
        "//modules/v2x/common:v2x_proxy_gflags",
        "//modules/v2x/proto:v2x_traffic_light_proto",
        "//modules/v2x/proto:v2x_carstatus_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/canbus/proto:canbus_proto",
        "//modules/map/hdmap:hdmap",
        "//modules/v2x/v2x_proxy/obu_interface:obu_interface_grpc_impl",
        "//modules/v2x/v2x_proxy/os_interface:os_interface",
    ],
)

cpplint()
