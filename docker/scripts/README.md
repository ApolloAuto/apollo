We define two Docker images: build and release.
The build image provides an environment where
`bash apollo.sh build` runs successfully,
the release image provides an environment where
`bash scripts/<module_name>.sh start` runs successfully.

The standard workflow for getting these containers
running is (from the main apollo directory):
```bash
bash docker/scripts/{dev/release}_start.sh
bash docker/scripts/{dev/release}_into.sh
```
Note that, within the scripts in this directory,
only standard tools that are expected
to be available in most Linux distributions
should be used (e.g., don't use realpath).
