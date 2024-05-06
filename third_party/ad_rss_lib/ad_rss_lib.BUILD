load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("@apollo//tools/install:install.bzl", "install", "install_files", "install_src_files")

install(
    name = "install",
    targets = [
        ":libad_rss.so",
    ],
    library_dest = "3rd-ad-rss-lib/lib",
    deps = [
        ":ad_rss_export_hdrs",
    ],
)
install_files(
    name = "ad_rss_export_hdrs",
    dest = "3rd-ad-rss-lib/include/ad_rss",
    files = glob([
        "include/ad_rss/**/*.hpp",
        "src/**/*.hpp",
        "include/generated/ad_rss/**/*.hpp"
    ]),
    strip_prefix = [
        "include/ad_rss",
        "src",
        "include/generated/ad_rss",
    ]
)

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])

cc_binary(
    name = "libad_rss.so",
    srcs = [
        "src/generated/physics/Acceleration.cpp",
        "src/generated/physics/CoordinateSystemAxis.cpp",
        "src/generated/physics/Distance.cpp",
        "src/generated/physics/DistanceSquared.cpp",
        "src/generated/physics/Duration.cpp",
        "src/generated/physics/DurationSquared.cpp",
        "src/generated/physics/ParametricValue.cpp",
        "src/generated/physics/Speed.cpp",
        "src/generated/physics/SpeedSquared.cpp",
        "src/generated/situation/LateralRelativePosition.cpp",
        "src/generated/situation/LongitudinalRelativePosition.cpp",
        "src/generated/situation/SituationType.cpp",
        "src/generated/state/LateralResponse.cpp",
        "src/generated/state/LongitudinalResponse.cpp",
        "src/generated/world/LaneDrivingDirection.cpp",
        "src/generated/world/LaneSegmentType.cpp",
        "src/generated/world/ObjectType.cpp",
        "src/core/RssCheck.cpp",
        "src/core/RssResponseResolving.cpp",
        "src/core/RssResponseTransformation.cpp",
        "src/core/RssSituationChecking.cpp",
        "src/core/RssSituationExtraction.cpp",
        "src/physics/Math.cpp",
        "src/situation/RSSFormulas.cpp",
        "src/situation/RssIntersectionChecker.cpp",
        "src/situation/RSSSituation.cpp",
        "src/world/RssSituationCoordinateSystemConversion.cpp",
        "src/world/RssObjectPositionExtractor.cpp",
    ] + glob(["src/**/*.hpp","include/**/*.hpp"]),
    copts = [
        "-fPIC",
        "-std=c++11",
        "-Werror",
        "-Wall",
        "-Wextra",
        "-pedantic",
        "-Wconversion",
        "-Wsign-conversion",
    ],
    includes = [
        "include",
        "include/generated",
        "src",
        "tests/test_support",
    ],
    linkshared = True,
)


cc_library(
    name = "ad_rss",
    srcs = ["libad_rss.so"],
    hdrs = glob([
        "include/**/*.hpp",
        "src/**/*.h*",
        "tests/test_support/**/*.h*",
    ]),
    copts = [
        "-fPIC",
        "-std=c++11",
        "-Werror",
        "-Wall",
        "-Wextra",
        "-pedantic",
        "-Wconversion",
        "-Wsign-conversion",
    ],
    includes = [
        "include",
        "include/generated",
        "src",
        "tests/test_support",
    ],
)


################################################################################
# Install section
################################################################################

################################################################################
# Doxygen documentation
################################################################################
