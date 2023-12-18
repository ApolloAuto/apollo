load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)
# install(
#     name = "install",
#     data_dest = "planning-task-rss-decider",
#     data = [
#         ":cyberfile.xml",
#         ":planning-task-rss-decider.BUILD",
#     ],
#     deps = [
#         "install_plugin",
#     ],
# )

# install_plugin(
#     name = "install_plugin",
#     data = [
#     ] + glob(["conf/**"]),
#     description = ":plugins.xml",
#     plugin = ":librss_decider.so",
# )

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_all_src",
#         ":install_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_all_src",
#     src_dir = ["."],
#     dest = "planning-task-rss-decider/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_hdrs",
#     src_dir = ["."],
#     dest = "planning-task-rss-decider/include",
#     filter = "*.h",
# )

apollo_cc_test(
    name = "rss_decider_test",
    size = "small",
    srcs = ["rss_decider_test.cc"],
    tags = ["exclude"],
    deps = [
        ":rss_decider_lib",
        "@ad_rss_lib//:ad_rss",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_plugin(
    name = "librss_decider.so",
    srcs = ["rss_decider.cc"],
    hdrs = ["rss_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//modules/common/status",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/rss_decider/proto:rss_decider_cc_proto",
        "@ad_rss_lib//:ad_rss",
    ],
)

apollo_package()

cpplint()
