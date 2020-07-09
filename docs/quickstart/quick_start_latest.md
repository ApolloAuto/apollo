# Apollo Master Quick Start Guide

The following guide serves as a user manual for launching the Apollo upgraded software and hardware stack on vehicle.

## Software Installation

### Start apollo-dev docker container

We start apollo-dev docker container following the convention in previous apollo releases.

```
bash docker/scripts/dev_start.sh
```

### Enter apollo-dev docker container
```
bash docker/scripts/dev_into.sh
```

### Configure custom settings
The first time you get into apollo-dev container named `apollo_dev_${USER}`,
please run apollo bootstrap command as follows:
```
bash ./apollo.sh config
```

> Experimental:
 There is also an *interactive* mode of bootstraping by running
 `bash apollo.sh config --interactive` or `bash apollo.sh config -i`.
 For now, this functionality is incomplete.

### Build modules

Run the following command to build all modules
```
bash apollo.sh build
```

Run the following to build individual modules
```
bash apollo.sh build cyber # OR
bash apollo.sh build planning # OR
bash apollo.sh build perception
```

By default, `apollo.sh build` generates debug outputs.

To run optimization build,  just substitute `build` with `build_opt` in the examples above.

### Test modules

```
bash apollo.sh test # To run unit-tests in all modules including cyber.
bash apollo.sh test cyber # run all unit-tests for cyber
bash apollo.sh test --config=gpu prediction # run all unit-test for the prediction module
```

### Build/Test: An Insider's View

Please note also that when it comes to build/test, `apollo.sh` runs
`scripts/apollo_build.sh` and `scripts/apollo_test.sh` respectively, both of
which are just simple wrapper around `bazel build` and `bazel test` commands.

So if you are familiar with bazel, you can run the following to build and test any targets:

```
bazel build //cyber/...
bazel test --config=gpu //modules/localization/...
bazel test //cyber/common:file_test
```

### Running lint

```
bash apollo.sh lint
```

### Bazel x Apollo

The bazel version pre-installed in our cyber/dev docker images are 3.2.0+.
And the default `--distdir` destination is under
`${APOLLO_ROOT_DIR}/.cache/distdir`.


