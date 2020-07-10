load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "libtorch",
    deps = select({
        "//tools/platform:use_gpu": [
            "@libtorch_gpu",
        ],
        "//conditions:default": [
            "@libtorch_cpu",
        ],
    }),
)

# systemlibs
# TODO(infra): all the system libs should have its own section

# libncurses5-dev
cc_library(
    name = "ncurses",
    linkopts = ["-lncurses"],
)

# libsqlite3-dev
cc_library(
    name = "sqlite3",
    linkopts = ["-lsqlite3"],
)

# uuid-dev
cc_library(
    name = "uuid",
    linkopts = ["-luuid"],
)

# atlas-cblas
cc_library(
    name = "cblas",
    linkopts = [
        "-lcblas",
    ],
)

cc_library(
    name = "zlib",
    linkopts = [
        "-lz",
    ],
)

cc_library(
    name = "console_bridge",
    linkopts = [
        "-lconsole_bridge",
    ],
)

cc_library(
    name = "glvnd",
    linkopts = [
        "-lGL",
    ],
)
