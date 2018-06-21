## How to Debug the Dreamview Start Problem

### Steps to Start Dreamview

If you encounter problems when starting Dreamview in the `docker/scripts/dev` sequence, first check if you are using the correct commands as shown below.

```bash
$ bash docker/scripts/dev_start.sh
$ bash docker/scripts/dev_into.sh
$ cd /apollo
$ bash apollo.sh build
$ bash scripts/dreamview.sh
```
### Dreamview Fails to Start

If Dreamview fails to start, use the script below to check the Dreamview startup log and restart Dreamview.

```bash
# Start Dreamview in foreground to see any error message it prints out during startup
$ bash scripts/dreamview.sh start_fe

# check dreamview startup log
$ cat data/log/dreamview.out
terminate called after throwing an instance of 'CivetException'
  what():  null context when constructing CivetServer. Possible problem binding to port.

$ sudo apt-get install psmisc

# to check if dreamview is running from other terminal
$ sudo lsof -i :8888

# kill other running/pending dreamview
$ sudo fuser -k 8888/tcp

# restart dreamview again
$ bash scripts/dreamview.sh
```

### Debug dreamview with gdb

If you get nothing in dreamview startup logs, you can try to debug dreamview with gdb, use the following commands:

```
$ gdb --args /apollo/bazel-bin/modules/dreamview/dreamview --flagfile=/apollo/modules/dreamview/conf/dreamview.conf
# or
$ source scripts/apollo_base.sh;
$ start_gdb dreamview
```

Once gdb is launched, press `r` and `enter` key to run,  if dreamview crashes, then get the backtrace with `bt`.

If you see an error `Illegal instruction` and something related with **libpcl_sample_consensus.so.1.7** in gdb backtrace, then you probably need to rebuild pcl lib from source by yourself and replace the one in the docker.

This usually happens when you're trying to run Apollo/dreamview on a machine that the CPU does not support FMA/FMA3 instructions, it will fail because the prebuilt pcl lib shipped with docker image is compiled with FMA/FMA3 support.
