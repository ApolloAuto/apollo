workspace(name = "apollo")

# googletest (GTest and GMock)
new_http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.8.0.tar.gz",
    sha256 = "58a6f4277ca2bc8565222b3bbd58a177609e9c488e8a72649359ba51450db7d8",
    build_file = "third_party/gtest.BUILD",
    strip_prefix = "googletest-release-1.8.0",
)

# gflags
http_archive(
    name = "com_github_gflags_gflags",
    url = "https://github.com/gflags/gflags/archive/v2.2.0.tar.gz",
    sha256 = "466c36c6508a451734e4f4d76825cf9cd9b8716d2b70ef36479ae40f08271f88",
    strip_prefix = "gflags-2.2.0",
)

bind(
    name = "gflags",
    actual = "@com_github_gflags_gflags//:gflags",
)

# glog
new_http_archive(
    name = "glog",
    url = "https://github.com/google/glog/archive/v0.3.5.tar.gz",
    sha256 = "7580e408a2c0b5a89ca214739978ce6ff480b5e7d8d7698a2aa92fadc484d1e0",
    build_file = "third_party/glog.BUILD",
    strip_prefix = "glog-0.3.5",
)

# Google Benchmark
new_http_archive(
    name = "benchmark",
    url = "https://github.com/google/benchmark/archive/v1.1.0.tar.gz",
    sha256 = "e7334dd254434c6668e33a54c8f839194c7c61840d52f4b6258eee28e9f3b20e",
    build_file = "third_party/benchmark.BUILD",
    strip_prefix = "benchmark-1.1.0"
)

# proto rules (Protobuf and GRPC)
http_archive(
    name = "org_pubref_rules_protobuf",
    url = "https://github.com/pubref/rules_protobuf/archive/v0.7.1.tar.gz",
    sha256 = "646b39438d8eeba02d9af890dee444c7e4e9d08ae8611bc0e0621257010162db",
    strip_prefix = "rules_protobuf-0.7.1",
)

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cpp_proto_repositories")
cpp_proto_repositories(
    lang_deps = {
        # Grpc repo is required by multiple languages but we put it here.
        "com_github_grpc_grpc": {
            "rule": "git_repository",
            "remote": "https://github.com/grpc/grpc.git",
            "init_submodules": True,
            "commit": "3808b6efe66b87269d43847bc113e94e2d3d28fb",
            #"tag": "v1.0.1",
        },

        # Hooray! The boringssl team provides a "master-with-bazel" branch
        # with all BUILD files ready to go.  To update, pick the
        # newest-ish commit-id off that branch.
        "boringssl": {
            "rule": "http_archive",
            "url": "https://github.com/google/boringssl/archive/master-with-bazel.zip"
        },

        # libssl is required for c++ grpc where it is expected in
        # //external:libssl.  This can be either boringssl or openssl.
        "libssl": {
            "rule": "bind",
            "actual": "@boringssl//boringssl-master-with-bazel:ssl",
        },

        # C-library for zlib
        "com_github_madler_zlib": {
            "rule": "new_git_repository",
            "remote": "https://github.com/madler/zlib",
            "tag": "v1.2.8",
            "build_file": "third_party/com_github_madler_zlib.BUILD",
        },

        # grpc++ expects //external:zlib
        "zlib": {
            "rule": "bind",
            "actual": "@com_github_madler_zlib//:zlib",
        },

        # grpc++ expects "//external:protobuf_clib"
        "protobuf_clib": {
            "rule": "bind",
            "actual": "@com_github_google_protobuf//:protobuf",
        },

        # grpc++ expects //external:nanopb
        "nanopb": {
            "rule": "bind",
            "actual": "@com_github_grpc_grpc//third_party/nanopb",
        },

        # Bind the executable cc_binary grpc plugin into
        # //external:protoc_gen_grpc_cpp.  Expects
        # //external:protobuf_compiler. TODO: is it really necessary to
        # bind it in external?
        "protoc_gen_grpc_cpp": {
            "rule": "bind",
            "actual": "@com_github_grpc_grpc//:grpc_cpp_plugin",
        },

        # Bind the protobuf proto_lib into //external.  Required for
        # compiling the protoc_gen_grpc plugin
        "protobuf_compiler": {
            "rule": "bind",
            "actual": "@com_github_google_protobuf//:protoc_lib",
        },

        # GTest is for our own internal cc tests.
        "gtest": {
            "rule": "new_git_repository",
            "remote": "https://github.com/google/googletest.git",
            "commit": "ed9d1e1ff92ce199de5ca2667a667cd0a368482a",
            "build_file": "third_party/protobuf_gtest.BUILD",
        },
    },
)

load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_repositories")
py_proto_repositories()

# cpplint from google style guide
new_git_repository(
    name = "google_styleguide",
    remote = "https://github.com/google/styleguide.git",
    commit = "159b4c81bbca97a9ca00f1195a37174388398a67",
    build_file = "third_party/google_styleguide.BUILD",
)

# eigen
new_http_archive(
    name = "eigen",
    url = "https://bitbucket.org/eigen/eigen/get/3.2.10.tar.gz",
    sha256 = "04f8a4fa4afedaae721c1a1c756afeea20d3cdef0ce3293982cf1c518f178502",
    build_file = "third_party/eigen.BUILD",
    strip_prefix = "eigen-eigen-b9cd8366d4e8",
)

# websocket++/websocketpp
new_http_archive(
    name = "websocketpp",
    url = "https://github.com/zaphoyd/websocketpp/archive/0.7.0.tar.gz",
    sha256 = "07b3364ad30cda022d91759d4b83ff902e1ebadb796969e58b59caa535a03923",
    build_file = "third_party/websocketpp.BUILD",
    strip_prefix = "websocketpp-0.7.0",
)

# CivetWeb (web server)
new_http_archive(
    name = "civetweb",
    url = "https://github.com/civetweb/civetweb/archive/v1.9.1.tar.gz",
    sha256 = "880d741724fd8de0ebc77bc5d98fa673ba44423dc4918361c3cd5cf80955e36d",
    build_file = "third_party/civetweb.BUILD",
    strip_prefix = "civetweb-1.9.1",
)

# curlpp
new_http_archive(
    name = "curlpp",
    url = "https://github.com/jpbarrette/curlpp/archive/v0.8.1.tar.gz",
    sha256 = "97e3819bdcffc3e4047b6ac57ca14e04af85380bd93afe314bee9dd5c7f46a0a",
    build_file = "third_party/curlpp.BUILD",
    strip_prefix = "curlpp-0.8.1",
)
