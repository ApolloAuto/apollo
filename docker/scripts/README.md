# Working with Docker containers

## Start and enter Apollo development container

The following commands are assumed to run from Apollo root directory.

```bash
bash docker/scripts/dev_start.sh
bash docker/scripts/dev_into.sh
```

## Start and enter multiple Apollo development containers for multiple users

Each Apollo development container must be run by each user separately. Switch to the desired user and run:

```bash
bash docker/scripts/dev_start_multiuser.sh
bash docker/scripts/dev_into.sh
```

Note:

> Multiple Apollo development containers are isolated now and can interact with host environment through exposed ports for Dreamview and Bridge.

### Start Dreamview in multiple Apollo development containers

Run the following command to start the Monitor module and Dreamview backend.

```bash
bash scripts/bootstrap.sh
```

_Output_

```bash
[ OK ] Launched module monitor.
[ OK ] Launched module dreamview.
[INFO] Dreamview is running at http://192.168.233.13:8890
```

The output of the above command will show which URL (`http://192.168.233.13:8890`) is being used for Dreamview.

### Start Bridge in multiple Apollo development containers

Run the following command to start the Bridge.

```bash
bash scripts/bridge.sh
```

_Output_

```bash
[INFO] Bridge port: 9092

```

The output of the above command will show which PORT (`9092`) is being used for Bridge.
