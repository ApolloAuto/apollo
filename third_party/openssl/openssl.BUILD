load("@rules_foreign_cc//foreign_cc:defs.bzl", "configure_make", "configure_make_variant")

# Read https://wiki.openssl.org/index.php/Compilation_and_Installation.

filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
)

CONFIGURE_OPTIONS = [
    "no-comp",
    "no-idea",
    "no-weak-ssl-ciphers",
    # "no-shared",
]

LIB_NAME = "openssl"

MAKE_TARGETS = [
    "build_libs",
    "install_dev",
]

alias(
    name = "openssl",
    actual = select({
        "//conditions:default": "openssl_default",
    }),
    visibility = ["//visibility:public"],
)

configure_make(
    name = "openssl_default",
    configure_command = "config",
    configure_in_place = True,
    configure_options = CONFIGURE_OPTIONS,
    env = select({
        "@platforms//os:macos": {"AR": ""},
        "//conditions:default": {},
    }),
    lib_name = LIB_NAME,
    lib_source = ":all_srcs",
    out_shared_libs = [
        "libssl.so",
        "libcrypto.so",
    ],
    # Note that for Linux builds libssl must be listed before
    # libcrypto on the linker command-line.
    out_static_libs = [
        "libssl.a",
        "libcrypto.a",
    ],
    targets = MAKE_TARGETS,
)

filegroup(
    name = "gen_dir",
    srcs = [":openssl"],
    output_group = "gen_dir",
    visibility = ["//visibility:public"],
)
