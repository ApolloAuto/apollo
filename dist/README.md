# Apollo Release Build

This directory contains Bazel rules for packaging Apollo binaries, static/shared
libraries, scripts, config/data files.

Rules should be grouped by module, like the following:

```text
apollo-dist
    - cyber-dist
    - perception-dist
    - prediction-dist
    - planning-dist
    - ...
```
