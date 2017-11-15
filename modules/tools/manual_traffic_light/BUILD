load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "manual_traffic_light",
    srcs = ["manual_traffic_light.cc"],
    data = ["//modules/map:map_data"],
    deps = [
        "//external:gflags",
        "//modules/common",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/common/util:string_util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:perception_proto",
    ],
)

cpplint()
