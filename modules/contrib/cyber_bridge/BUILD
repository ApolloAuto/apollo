load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package", "apollo_cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "cyber_bridge",
    srcs = ["bridge.cc"],
    deps = [
        ":contrib_cyber_bridge",
        "@boost",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_library(
    name = "contrib_cyber_bridge",
    srcs = [
        "client.cc",
        "clients.cc",
        "node.cc",
        "server.cc",
    ],
    hdrs = [
        "client.h",
        "clients.h",
        "node.h",
        "server.h",
    ],
    deps = [
        "//cyber",
        "@boost",
    ],
)

apollo_package()

cpplint()
