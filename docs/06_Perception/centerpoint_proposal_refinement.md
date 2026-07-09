# CenterPoint Proposal Refinement Extension

This document describes an optional CenterPoint proposal refinement extension
point for LiDAR detection. The extension is disabled by default and preserves
the original CenterPoint behavior unless explicitly enabled in the detector
model config.

## Scope

The extension point runs after CenterPoint proposal decode and before the
existing object construction and NMS/filtering path.

```text
CenterPoint preprocess / inference / decode
  -> optional proposal refinement
  -> existing object construction / NMS / filtering
  -> existing perception output
```

It does not change Cyber RT input or output channels, perception message
semantics, tracking, fusion, prediction, or planning.

## Configuration

The extension is controlled by `ProposalRefineParam` in
`modules/perception/lidar_detection/detector/center_point_detection/proto/model_param.proto`.

Important fields:

```text
enable_proposal_refine: false
proposal_refine_mode: "mock_zero"
proposal_refine_top_k: -1
proposal_refine_score_threshold: 0.0
proposal_refine_delta_scale: 1.0
proposal_refine_debug_log: false
strict_refine: true
```

When `enable_proposal_refine=false`, the extension does not run and the original
CenterPoint path is preserved.

## Modes

`mock_zero` is a no-op mode. It selects proposals, records statistics, and
leaves boxes, scores, and labels unchanged.

`mock_delta` applies a deterministic small delta for test coverage:

```text
delta_x = 0.1
delta_yaw = 0.05
delta_score = 0.01
```

Both modes are intended for validating the extension point, configuration
gating, delta application, and rollback behavior. They are not intended to claim
perception quality improvement.

## Safety

- Disabled by default.
- No new runtime dependency on quantum libraries.
- No Cyber RT channel changes.
- No public perception message changes.
- No tracking/fusion/prediction/planning changes.
- Box dimensions are clamped positive after delta application.
- Yaw is normalized.
- Score is clamped to `[0, 1]`.

## Rollback

Set:

```text
enable_proposal_refine: false
```

or remove the optional `proposal_refine` config block. With refinement disabled,
the module skips proposal conversion/refinement and continues through the
original CenterPoint detection path.

## Tests

Focused tests:

```bash
bazel test --config=unit_test --cache_test_results=no --test_output=errors \
  //modules/perception/lidar_detection:proposal_refine_test
```

Focused build:

```bash
bazel build --config=opt --config=gpu --config=nvidia \
  //modules/perception/lidar_detection:apollo_perception_lidar_detection \
  //modules/perception/lidar_detection:liblidar_detection_component.so
```

Lint targets:

```bash
bazel build \
  //modules/perception/lidar_detection:center_point_proposal_refine_cpplint \
  //modules/perception/lidar_detection:proposal_refine_test_cpplint
```

## Follow-Up Work

Optional proposal export, offline teacher experiments, student-model inference,
and experiment reports should be split into later PRs. They are intentionally
outside the conservative first PR scope.
