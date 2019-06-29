# Data Pipeline Proxy

Data is an integral part of autonomous driving the ego car collects GBs of data via sensors, scenarios and trigger events which can be applied to improving the technology and creating models to help in the training and development of our algorithms.
We have offered a unique opportunity through the Data Pipeline tool to communicate with our cloud service and harness the power of data which is open to our partners and all of this performance can be tested using our in-built dashboard.

## Features
1. **Intelligent data collection** - Custom events driven with a highly configurable acquisition mode and visual collection tasks
2. **Large-scale cloud processing** - Leverage the power of cloud for efficient task scheduling, use our powerful cloud computing resources to create precise artificial intelligence models.
3. **Custom simulation verification** - Highly configurable environment with instant verification on the cloud


## Prerequisite

1. You should have your own S3-compatible cloud storage. Currently we support:

   * [Baidu BOS](https://cloud.baidu.com/doc/BOS/index.html)

## Submit Job

1. Prepare a [job config](proto/job_config.proto) file. See the
   [example](conf/example_job.pb.txt).

1. Build Apollo Python libs in /apollo.

   ```bash
   ./apollo.sh build_py
   ```

1. Activate the Python 3 Conda environment, and run the tool.

   ```bash
   source activate py37
   python submit_job.py --job_config=/path/to/your/job.pb.txt
   ```
