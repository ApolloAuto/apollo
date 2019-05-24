# Apollo Fuel Proxy

A tool to communicate with Apollo Fuel cloud service, which is open to our
partners.

## Prerequisite

1. Install and activate the corresponding Conda environment.

   ```bash
   conda env update -f conda-py37.yaml
   source activate fuel-proxy
   ```

1. You should have your own S3-compatible cloud storage. Currently we support:

   * [Baidu BOS](https://cloud.baidu.com/doc/BOS/index.html)
