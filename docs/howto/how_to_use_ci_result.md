# How to Use CI Result in Apollo
In Apollo, wether a PR can be merged depends on CI result and CLA.

## What works will CI check?
Apollo CI will follow the steps below:

1. Merge your PR into master and build
1. Lint your code including .cc, .h, .py, BUILD, etc
1. Run all unit tests in apollo project

So it's recommended that run

```
./apollo.sh lint
./apollo.sh build
./apollo.sh test
```
before commiting your code.

When your PR CI fails, you can click `Details`

![build_failed](images/build_failed.png)

Now you are in our CI system, enter `Build Log` to see detailed fail log.

![detail_log](images/build_log.png)

## Possible Errors And Solution

### Error: "FAIL: //modules/perception/base:blob_cpplint"

![lint](images/lint.png)

This is due to lint error. Apollo adopted Google coding style, so the header files shoud be in the suggested order. If you can't find the suggestion, please turn the log up and seek carefully.

### Error: "FAIL: //modules/perception/base:blob_test"

![test_failed](images/unit_test_failed.png)

![test_failed_log](images/unit_failed_log.png)

This is due to a unit test failure. You can correct the unit test according to the log. Especially when timeout happens, you can try changing the `size` filed in BUILD from `small` to `medium` or `large`, hope it works.

If more complicated situation happens, please feel free to commonent in your PR.
