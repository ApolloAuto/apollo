workspace(name = "apollo")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Common rules with bazel_federation:
# https://github.com/bazelbuild/bazel-federation
http_archive(
    name = "bazel_federation",
    url = "file:///home/libs/bazel-federation-0.0.1.tar.gz",
    sha256 = "e9326b089c10b2a641099b1c366788f7df7c714ff71495a70f45b20c4fe1b521",
    strip_prefix = "bazel-federation-0.0.1",
)
load("@bazel_federation//:repositories.bzl", "rules_cc")
rules_cc()
load("@bazel_federation//setup:rules_cc.bzl", "rules_cc_setup")
rules_cc_setup()

# Proto rules: https://github.com/bazelbuild/rules_proto
http_archive(
    name = "rules_proto",
    # 2019-08-01
    url = "file:///home/libs/rules_proto-97d8af4dc474595af3900dd85cb3a29ad28cc313.tar.gz",
    sha256 = "602e7161d9195e50246177e7c55b2f39950a9cf7366f74ed5f22fd45750cd208",
    strip_prefix = "rules_proto-97d8af4dc474595af3900dd85cb3a29ad28cc313",
)
load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies", "rules_proto_toolchains")
rules_proto_dependencies()
rules_proto_toolchains()

# cpplint from google style guide
http_archive(
    name = "google_styleguide",
    # 2020-01-02
    url = "file:///home/libs/styleguide-159b4c81bbca97a9ca00f1195a37174388398a67.tar.gz",
    sha256 = "3ed86946e6e637f0fe21749c0323b086e62c4b8b93694d6cedad615cdc584512",
    strip_prefix = "styleguide-159b4c81bbca97a9ca00f1195a37174388398a67",
    build_file = "google_styleguide.BUILD",
)

http_archive(
    name = "com_google_absl",
    url = "file:///home/libs/abseil-cpp-20190808.tar.gz",
    sha256 = "8100085dada279bf3ee00cd064d43b5f55e5d913be0dfe2906f06f8f28d5b37e",
    strip_prefix = "abseil-cpp-20190808",
)

http_archive(
    name = "com_google_googletest",
    url = "file:///home/libs/googletest-release-1.10.0.tar.gz",
    sha256 = "9dc9157a9a1551ec7a7e43daea9a694a0bb5fb8bec81235d8a1e6ef64c716dcb",
    strip_prefix = "googletest-release-1.10.0",
)

http_archive(
    name = "com_github_gflags_gflags",
    url = "file:///home/libs/gflags-2.2.2.tar.gz",
    sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",
    strip_prefix = "gflags-2.2.2",
)

http_archive(
    name = "com_google_glog",
    url = "file:///home/libs/glog-0.4.0.tar.gz",
    sha256 = "f28359aeba12f30d73d9e4711ef356dc842886968112162bc73002645139c39c",
    strip_prefix = "glog-0.4.0",
)

http_archive(
    name = "com_github_grpc_grpc",
    url = "file:///home/libs/grpc-1.26.0.tar.gz",
    sha256 = "2fcb7f1ab160d6fd3aaade64520be3e5446fc4c6fa7ba6581afdc4e26094bd81",
    strip_prefix = "grpc-1.26.0",
)

http_archive(
    name = "eigen",
    url = "file:///home/libs/eigen-3.2.10.tar.gz",
    sha256 = "04f8a4fa4afedaae721c1a1c756afeea20d3cdef0ce3293982cf1c518f178502",
    build_file = "eigen.BUILD",
    strip_prefix = "eigen-eigen-b9cd8366d4e8",
)

http_archive(
    name = "qpOASES",
    url = "file:///home/libs/qp-oases-3.2.1-1.zip",
    sha256 = "e70b49586b58b8f5fd348e951f3c3094ed0ad371a96097a499f343a7aeec7dbe",
    build_file = "qpOASES.BUILD",
    strip_prefix = "qp-oases-3.2.1-1",
)

http_archive(
    name = "osqp",
    url = "file:///home/libs/osqp-0.4.1.zip",
    sha256 = "3e431342bbe3bd578f1ef768e8dff1f89d2809d8195aa15f8ed72628ca92f6d8",
    build_file = "osqp.BUILD",
    strip_prefix = "osqp-contrib-master",
)

http_archive(
    name = "com_github_jbeder_yaml_cpp",
    url = "file:///home/libs/yaml-cpp-587b24e2eedea1afa21d79419008ca5f7bda3bf4.tar.gz",
    sha256 = "e0d2f5d513e334f890ed430e140eb998de363e1299aa24fef589266ea75c6e9d",
    strip_prefix = "yaml-cpp-587b24e2eedea1afa21d79419008ca5f7bda3bf4",
)

http_archive(
    name = "civetweb",
    url = "file:///home/libs/civetweb-1.11.tar.gz",
    sha256 = "de7d5e7a2d9551d325898c71e41d437d5f7b51e754b242af897f7be96e713a42",
    build_file = "civetweb.BUILD",
    strip_prefix = "civetweb-1.11",
)

http_archive(
    name = "proj4",
    url = "file:///home/libs/proj.4-4.9.3.zip",
    sha256 = "9d6d845ae77928441631882e25177117534dbe4311b823ee35eb100d3b69a78e",
    build_file = "proj4.BUILD",
    strip_prefix = "PROJ-4.9.3",
)

http_archive(
    name = "adv_plat",
    url = "https://github.com/ApolloAuto/apollo-contrib/releases/download/v3.0.0/plat-sw-3.0.0.1.zip",
    sha256 = "0a58dadab924b520d5b5a58ef82fc0f76c2aa4feaaabd49ec9873228c125d513",
    build_file = "adv_plat.BUILD",
)

http_archive(
    name = "tinyxml2",
    url = "file:///home/libs/tinyxml2-5.0.1.zip",
    sha256 = "f74379e9c3942f539f2236cf0e57edd11f8da684dc3000ded77304c18b1dad1d",
    build_file = "tinyxml2.BUILD",
    strip_prefix = "tinyxml2-5.0.1",
)

http_archive(
    name = "ad_rss_lib",
    url = "file:///home/libs/ad-rss-lib-1.1.0.tar.gz",
    build_file = "external/rss_lib.BUILD",
    strip_prefix = "ad-rss-lib-1.1.0",
)

new_local_repository(
    name = "local_integ",
    build_file = "external/local_integ.BUILD",
    path = "/usr/local/apollo/local_integ",
)

new_local_repository(
    name = "pytorch",
    build_file = "external/pytorch.BUILD",
    path = "/usr/local/miniconda/envs/py3c/lib/python3.6/site-packages/torch",
)

new_local_repository(
    name = "pytorch_gpu",
    build_file = "external/pytorch_gpu.BUILD",
    path = "/usr/local/miniconda/envs/py3g/lib/python3.6/site-packages/torch",
)
