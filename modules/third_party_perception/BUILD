load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "third_party_perception_lib",
    srcs = ["third_party_perception_base.cc"],
    hdrs = ["third_party_perception_base.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/third_party_perception/tools:third_party_perception_filter",
        "//modules/third_party_perception/tools:third_party_perception_fusion",
        "//modules/third_party_perception/tools:third_party_perception_mobileye_conversion",
        "//modules/third_party_perception/tools:third_party_perception_radar_conversion",
        "//modules/third_party_perception/tools:third_party_perception_smartereye_conversion",
    ],
)

cc_library(
    name = "third_party_perception_mobileye_lib",
    srcs = ["third_party_perception_mobileye.cc"],
    hdrs = ["third_party_perception_mobileye.h"],
    alwayslink = True,
    deps = [
        ":third_party_perception_lib",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/third_party_perception/tools:third_party_perception_filter",
        "//modules/third_party_perception/tools:third_party_perception_fusion",
        "//modules/third_party_perception/tools:third_party_perception_mobileye_conversion",
        "//modules/third_party_perception/tools:third_party_perception_radar_conversion",
        "//modules/third_party_perception/tools:third_party_perception_smartereye_conversion",
    ],
)

cc_library(
    name = "third_party_perception_smartereye_lib",
    srcs = ["third_party_perception_smartereye.cc"],
    hdrs = ["third_party_perception_smartereye.h"],
    alwayslink = True,
    deps = [
        ":third_party_perception_lib",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/third_party_perception/tools:third_party_perception_filter",
        "//modules/third_party_perception/tools:third_party_perception_fusion",
        "//modules/third_party_perception/tools:third_party_perception_mobileye_conversion",
        "//modules/third_party_perception/tools:third_party_perception_radar_conversion",
        "//modules/third_party_perception/tools:third_party_perception_smartereye_conversion",
    ],
)

cc_library(
    name = "third_party_perception_component_lib",
    srcs = ["third_party_perception_component.cc"],
    hdrs = ["third_party_perception_component.h"],
    copts = ['-DMODULE_NAME=\\"third_party_perception\\"'],
    deps = [
        ":third_party_perception_lib",
        ":third_party_perception_mobileye_lib",
        ":third_party_perception_smartereye_lib",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libthird_party_perception_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":third_party_perception_component_lib"],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    library_dest = "third-party-perception/lib",
    data_dest = "third-party-perception",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":third-party-perception.BUILD",
    ],
    targets = [
        ":libthird_party_perception_component.so",
    ],
    deps = [
        ":pb_hdrs",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "third-party-perception/include",
    data = [
        "//modules/third_party_perception/proto:radar_obstacle_cc_proto",
        "//modules/third_party_perception/proto:third_party_perception_component_cc_proto",
    ],
)
install_src_files(
    name = "install_src",
    deps = [
        ":install_third-party-perception_src",
        ":install_third-party-perception_hdrs"
    ],
)

install_src_files(
    name = "install_third-party-perception_src",
    src_dir = ["."],
    dest = "third-party-perception/src",
    filter = "*",
)

install_src_files(
    name = "install_third-party-perception_hdrs",
    src_dir = ["."],
    dest = "third-party-perception/include",
    filter = "*.h",
)
cpplint()
