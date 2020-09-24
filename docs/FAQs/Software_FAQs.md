# Software FAQs

## Can other operating systems besides Ubuntu be used?

We have only tested on Ubuntu which means it's the only operating system we
currently officially support. Users are always welcome to try different
operating systems and can share their patches with the community if they are
successfully able to use them.

---

## I’m having difficulty connecting to localhost:8888 (Dreamview).

The Dreamview web server is provided by the dreamview node(A node is an
executable in ROS concept). Before accessing the Dreamview page, you need to
build the system(including dreamview node) within the docker container following
the
[Software Installation Guide](../quickstart/apollo_software_installation_guide.md).
Once built, dreamview node will be started after the step
`bash scripts/bootstrap.sh`.

So if you can not access Dreamview, please check:

- Make sure you have dreamview process running correctly. In the latest version,
  `bash scripts/bootstrap.sh` will report `dreamview: ERROR (spawn error)` if
  dreamview fails to start. For early version, please check with command:
  `supervisorctl status dreamview` or `ps aux | grep dreamview`. If dreamview is
  not running, please refer to
  [How to Debug a Dreamview Start Problem](../howto/how_to_debug_dreamview_start_problem.md).
- Make sure the address and port are not blocked by the firewall.
- Make sure you're using <apollo_host_ip>:8888 instead of localhost:8888 if you
  are not accessing the Dreamview page through the host machine.

---

## How can I perform step-by-step debugging?

The majority of bugs can be found through logging (using AERROR, AINFO, ADEBUG).
If step-by-step debugging is needed, we recommend using gdb.

---

## How do I run the Offline Perception Visualizer?

Refer to the How-To guide located
[here](../howto/how_to_run_offline_perception_visualizer.md).

---

## How do I add a new module

Apollo currently functions as a single system, therefore before adding a module
to it, understand that there would be a lot of additional work to be done to
ensure that the module functions perfectly with the other modules of Apollo.
Simply add your module to the `modules/` folder. You can
use `modules/routing` as an example, which is a relatively simple module. Write
the BUILD files properly and apollo.sh will build your module automatically

---

## Build error "docker: Error response from daemon: failed to copy files: userspace copy failed":

An error message like this means that your system does not have enough space to
build Apollo and the build process will fail. To resolve this issue, run the
following to free up some space:

```
docker/setup_host/cleanup_resources.sh
```

If it does not work, delete the Apollo repo, free up some space and then try
again.

---

## Bootstrap error: unix:///tmp/supervisor.sock refused connection

There could be a number of reasons why this error occurs. Please follow the
steps recommended in the
[following thread](https://github.com/ApolloAuto/apollo/issues/5344). There are
quite a few suggestions. If it still does not work for you, comment on the
thread mentioned above.

---

## My OS keeps freezing when building Apollo 3.5?

If you see an error like this, you do not have enough memory to build Apollo.
Please ensure that you have at least **16GB** memory available before building
Apollo. You could also find `--jobs=$(nproc)` in apollo.sh file and replace it
with `--jobs=2`. This will make build process to use only 2 cores. Building will
be longer, but will use less memory.

---

**More Software FAQs to follow.**
