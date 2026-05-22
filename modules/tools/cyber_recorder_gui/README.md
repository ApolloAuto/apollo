# Cyber Recorder GUI Lite

`cyber_recorder_gui_lite` is a lightweight Qt graphical interface for replaying Apollo Cyber record files. It can open one record file or all record shards in a directory, display basic record information, and publish the recorded messages back to Cyber for debugging.

This module is the reduced version of `modules/cyber_recorder_gui`. It is intentionally kept as a separate package and executable so that both versions can coexist without an install-name conflict.

## Features

- Open a single `.record` file.
- Open a directory containing `.record` and `.record.*` shards.
- Preview record metadata and channel information.
- Play, pause, resume, and stop playback.
- Seek during playback with the progress bar.
- Display the selected time range and current playback status.

The lite version does not include the full version's map selector or `log2worldsim` export workflow.

## Build

From the Apollo workspace root (`/apollo_workspace` inside the development container):

```bash
# Apollo buildtool workflow
buildtool build -p modules/tools/cyber_recorder_gui --cpu
```

For a direct Bazel build (after the Apollo dependencies are installed):

```bash
bazel build //modules/tools/cyber_recorder_gui:cyber_recorder_gui_lite
```

The generated executable is:

```text
bazel-bin/modules/tools/cyber_recorder_gui/cyber_recorder_gui_lite
```

## Run

Run the executable directly:

```bash
./bazel-bin/modules/tools/cyber_recorder_gui/cyber_recorder_gui_lite
```

Or start it with `cyber_launch`:

```bash
cyber_launch start modules/tools/cyber_recorder_gui/launch/cyber_recorder_gui_lite.launch
```

The application initializes the Cyber node `cyber_recorder_gui_lite` and shows a window titled **Cyber Recorder GUI Lite**.

## Basic usage

1. Click **File** and select one record file, or click **Dir** and select a directory.
2. Check the metadata shown in the information panel.
3. Click **Play** to start replay.
4. Use **Pause** / **Resume** to control playback.
5. Drag or click the upper progress bar to seek while playing.
6. Click **Stop** to reset playback to the beginning.

The directory picker starts at `/apollo_workspace/data/`. Directory playback loads files matching `*.record` and `*.record.*`, sorted by file name. Keep record shards in a naturally sortable sequence, for example:

```text
demo.record
demo.record.00000
demo.record.00001
```

## Playback defaults

| Setting | Default |
| --- | --- |
| Playback rate | `1.0` |
| Start time | Record beginning |
| End time | Record end (`uint64_t` maximum is used as the default bound) |
| Preload time | `3` seconds |
| Loop playback | Disabled |
| Channel selection | All channels when no filter is configured |

The lower dual-handle slider currently updates the displayed `Range` label only; it does not restrict the actual playback interval.

## Troubleshooting

### No window appears

Make sure the container has access to the host display (X11 or Wayland) and that Qt can connect to it.

### A record cannot be opened

Verify that the path points to an Apollo Cyber record, that the file is not corrupted, and that the current user can read it. You can inspect a file with Apollo's record tools before starting the GUI.

### Other modules receive no messages

Confirm that the record contains the expected channel and that the subscriber is running in the same Cyber environment. The following commands can help:

```bash
cyber_channel list
cyber_channel echo /your/channel/name
```

### The progress bar does not seek

Seeking is enabled while playback is active. Start playback before dragging or clicking the upper progress bar.

## Source layout

| Path | Purpose |
| --- | --- |
| `main.cpp` | Qt application entry point and Cyber initialization |
| `mainwindow.ui` | Main window layout |
| `mainwindow.cpp` / `mainwindow.h` | File selection, playback controls, and status updates |
| `info.cc` / `info.h` | Record metadata reader |
| `player/` | Playback tasks, buffering, and message publishing |
| `launch/cyber_recorder_gui_lite.launch` | `cyber_launch` configuration |
| `BUILD` | Bazel targets and package installation rule |
| `cyberfile.xml` | Apollo package metadata (`cyber-recorder-gui-lite`) |
