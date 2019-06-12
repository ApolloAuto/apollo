# Apollo Fuel Proxy

A tool to communicate with Apollo Fuel cloud service, which is open to our
partners.

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
