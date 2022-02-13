
# OpenBolty – Apollo

## Setup Dev Environment

```bash
bash docker/scripts/dev_start.sh
bash docker/scripts/dev_into.sh

# once inside the container...
./apollo.sh build_opt_gpu
bash scripts/bootstrap.sh start
```

Then visit `http://localhost:8888` to see the Dreamview UI. See [here](docs/howto/how_to_launch_and_run_apollo.md) for instructions on how to use the Dreamview UI.
