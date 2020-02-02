# Docker image build process

## Usage

Simply run
```bash
./build_dev.sh ./dev.x86_64.dockerfile
```

## Understand the dependencies

To help users to understand the dependencies, we are trying to compile a high
level [dependency graph](dependencies.dot). You can build your own Dockerfile or
ISO package according to it.

To view the graph, please run:

```bash
sudo apt install xdot
xdot dependencies.dot
```

## Add new dependency

When you need to add something to the Docker image or the bazel WORKSPACE, it's
defined as a new dependency. Before doing that, please add it as well as all its
dependencies to the dependencies.dot first, so we understand all the
dependencies and dependents, which helps us manage its lifecycle like upgrading
and retiring in the future.

## Add new installer

The best practice of a new installer would be:

1. Well tested.

   Of course. Make it work, and don't break other installers, such as
   incompatible versions of libraries.

1. Standalone.

   Have minimum assumption about the basement, which means, you can depend on
   the base image and installers/pre_install.sh. Other than that, you should
   install all the dependencies in your own installer.

1. Thin.

   It will generate a new layer in the final image, so please keep it as thin as
   possible. For example, clean up all intermediate files:

   ```bash
   wget xxx.zip
   # Unzip, make, make install
   rm -fr xxx.zip xxx
   ```

1. Cross-architecture.

   It would be awesome to work perfectly for different architectures such as X86
   and ARM.
