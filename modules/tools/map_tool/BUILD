load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_tool",
    srcs = ["map_tool.cc"],
    data = ["//modules/map:map_data"],
    deps = [
        "//external:gflags",
        "//modules/common",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        
    ],
)

cpplint()
