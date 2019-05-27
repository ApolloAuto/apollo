package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])

cc_library(
    name = "ad_rss",
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
    ] + glob(["src/**/*.hpp"]),
    hdrs = glob(["include/**/*.hpp"]),
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
