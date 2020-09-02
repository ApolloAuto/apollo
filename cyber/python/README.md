# Python3 Wrapper for Cyber RT

## Usage

Make sure you have built Apollo successfully. Then in Python code:

```python3
import sys

from cyber.python.cyber_py3 import cyber


cyber.init()

if not cyber.ok():
    print('Well, something is wrong.')
    sys.exit(1)

# Do your job here.
cyber.shutdown()
```

Learn more usage from the [examples](cyber_py3/examples/) and
[tests](cyber_py3/test/).

**Note:** Like C++ files, Python files are also managed by Bazel since Apollo 6.0. You can take
[how_to_build_and_run_python_app.md](../../docs/howto/how_to_build_and_run_python_app.md)
as reference.
