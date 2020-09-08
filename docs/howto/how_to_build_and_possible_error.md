# How to Build Apollo And Possible Error

## Install Docker

Please follow the
[Apollo Software Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_software_installation_guide.md#Set-up-the-Docker-environment).

## Build Apollo

### Start container

We provide a build x86 image named dev-x86_64-18.04-\*. The Container will mount
your local apollo repo to /apollo.

```bash
bash docker/scripts/dev_start.sh
```

### Get into the container

```bash
bash docker/scripts/dev_into.sh
```

### Build modules

```bash
bash apollo.sh build [OPTION]
```

if you want to build apollo in diversified modules, you can choose 'build_dbg'、
'build_opt' etc.

`Note:` If you do not have a GPU, you can use the following script instead

```bash
bash apollo.sh build_cpu
```

some special build modules:

`build_dbg [module]`: run debug build.

`build_opt [module]`: run optimized build.

`build_cpu [module]`: build in CPU mode. Equivalent to 'bazel build
--config=cpu'

`build_gpu [module]`: run build in GPU mode. Equivalent to 'bazel build
--config=gpu'

`build_opt_gpu [module]`: optimized build in GPU mode. Equivalent to 'bazel
build --config=opt --config=gpu'

## Possible Errors And Solution

### Error: "ERROR: query interrupted" encountered during compilation

This is due to inconsistencies in the bazel internal cache.

**Solution:** Press any key to exit the compilation process. Execute the
following command to enter the Docker environment:

```bash
bash docker/scripts/dev_into.sh
```

Enter the following command in the Docker environment to perform the bazel
cleanup cache task (must keep the network unblocked in order to successfully
download the dependencies, otherwise the command will not work even if it is
executed multiple times):

```bash
bazel query //...
```

Finally, enter the `exit` command to exit the Docker environment and
`bash docker/scripts/dev_start.sh` to re-execute the build task.

### Staying in the "Building: no action running" interface for a long time during compile time

This is due to the existence of multiple different versions of Docker or bazel
internal cache inconsistency in the current system.

**Solution:** Press the shortcut key **Ctrl+C** to terminate the current build
process. Then use any of the following methods to stop:

1. Stop all current running processes in Docker

   ```bash
   docker stop $(docker ps -a | grep apollo | awk '{print $1}')
   ```

2. Stop all instances of Docker

   ```bash
   docker stop $(docker ps -aq)
   ```

3. Excecute command `./apollo.sh clean`
4. Once complete, `./apollo.sh build` to rebuild the Apollo project.

### Error: "Another command (pid=2466) is running. Waiting for it to complete..." appears at compile time

This is caused by compiling in other command line terminals or by pressing the
`Ctrl+C` key during the previous compilation, causing the compiling process'
execution to only partially complete.

**Solution:** Press `Ctrl+C` to terminate the current build process and follow
the steps below to terminate the residual build process:

1. Enter Docker

   ```bash
   bash docker/scripts/dev_into.sh
   ```

2. Kill the remaining compilation process in Docker

   ```bash
   pkill bazel-real
   ```

3. Check if the bazel-real process remains in the Docker. If yes, press `q` to
   exit and perform step 2 again. Also use `ps aux | grep bazel-real` to view
4. Exit Docker using

   ```bash
   exit
   ```

5. Use command `./apollo.sh build` in the terminal to re-execute the build task.
