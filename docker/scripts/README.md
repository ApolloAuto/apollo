We defined an development docker image, which provides an environment where
command `bash apollo.sh build` runs successfully,

The standard workflow for getting the development container
running is (from the main apollo directory):
```bash
bash docker/scripts/dev_start.sh
bash docker/scripts/dev_into.sh
```
Note that, within the scripts in this directory,
only standard tools that are expected
to be available in most Linux distributions
should be used (e.g., don't use realpath).
