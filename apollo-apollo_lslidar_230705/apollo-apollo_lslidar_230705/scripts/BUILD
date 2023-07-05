load("//tools/install:install.bzl", "install")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install_scripts",
    data = [
        ":apollo_base",
        ":bootstrap",
        ":bridge",
        ":docker_utils",
        ":map_generation",
        ":recorder",
        ":tools",
    ],
    data_dest = "scripts/src",
)

install(
    name = "install",
    data = [":cyberfile.xml"],
    data_dest = "scripts",
    deps = ["install_scripts"],
)

filegroup(
    name = "tools",
    srcs = [
        "localization_online_visualizer.sh",
        "msf_simple_map_creator.sh",
        "rtk_player.sh",
        "rtk_recorder.sh",
        "record_message.py",
        "record_message.sh",
    ],
)

filegroup(
    name = "bootstrap",
    srcs = [
        "bootstrap.sh",
        "bootstrap_lgsvl.sh",
        "dreamview.sh",
        "monitor.sh",
    ],
)

filegroup(
    name = "recorder",
    srcs = [
        "cyberfile.xml",
        "record_bag.py",
        "record_bag.sh",
    ],
)

filegroup(
    name = "bridge",
    srcs = [
        "bridge.sh",
    ],
)

filegroup(
    name = "docker_utils",
    srcs = [
        ":docker_start_user.sh",
    ],
)

filegroup(
    name = "map_generation",
    srcs = [
        ":create_map_from_mobileye.sh",
        ":create_map_from_xy.sh",
        ":generate_routing_topo_graph.sh",
    ],
)

filegroup(
    name = "apollo_base",
    srcs = [
        "apollo.bashrc",
        "apollo_base.sh",
        "common.bashrc",
    ],
)
