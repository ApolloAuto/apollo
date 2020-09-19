# How to Use CI Result in Apollo
Apollo PR can be merged depend on CI result and CLA.

## What works will CI check?
Apollo CI will run following steps:

1. Build your PR code merged master
1. Lint your PR code include cc, h, py, BUILD, etc
1. Run all unit tests in apollo project

So we recommend you can run

```
./apollo.sh lint
./apollo.sh build
./apollo.sh test
```
before commit your code. 

When the CI failed of you PR, you can enter `Details`

![build_failed](images/build_failed.png)

Now you are coming into our CI system, enter `Build Log` to see detail fail log.

![detail_log](images/build_log.png)

## Possible Errors And Solution

### Error: "FAIL: //modules/perception/base:blob_cpplint"

![lint](images/lint.png)

This is due to lint error, we obey Google code style. So the header file shoud be reordered followed the suggestion. If you can't find the suggestion, please turn the log up and seek carefully.

### Error: "FAIL: //modules/perception/base:blob_test"

![test_failed](images/unit_test_failed.png)

![test_failed_log](images/unit_failed_log.png)

This is due to unit test fail. Read the test log carefully, you can correct the unit test. Specially timeout happened, you can try change BUILD size small to large, hope it works.

If more complicated situation happened, welcome commonent in your PR.
