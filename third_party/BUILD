package(default_visibility = ["//visibility:public"])

# Libs provided by the system.

cc_library(
    name = "adolc",
    deps = [
        "@adolc",
    ],
)

# libboost-dev
# TODO(xiaoxq): This is not a recommended way to introduce boost. Instead, we'd
# better leverage things like https://github.com/nelhage/rules_boost
# However, another library, libpcl-dev depends on libboost-dev. So we decide to
# reuse it instead of making duplication.
cc_library(
    name = "boost",
    deps = ["@boost"],
)

cc_library(
    name = "ipopt",
    deps = [
        "@ipopt",
    ],
)

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

# libncurses5-dev
cc_library(
    name = "ncurses",
    linkopts = ["-lncurses"],
)

cc_library(
    name = "pthread",
    linkopts = ["-pthread"],
)

# python3-dev
cc_library(
    name = "python3",
    linkopts = ["-lpython3.6m"],
)

cc_library(
    name = "rt",
    linkopts = ["-lrt"],
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
