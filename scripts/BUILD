load("//tools/install:install.bzl", "install")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data = [
        ":apollo_base",
        ":bootstrap",
        ":bridge",
        ":docker_utils",
        ":map_generation",
        ":recorder",
        ":tools",
    ],
)

filegroup(
    name = "tools",
    srcs = [
        "rtk_player.sh",
        "rtk_recorder.sh",
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
