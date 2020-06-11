# Python3 Wrapper for Cyber RT

## Usage

Make sure you have built Apollo successfully, which should also have added
`/apollo/cyber/python` to the PYTHONPATH for you. Then in Python code:

```python3
import sys

from cyber_py3 import cyber


cyber.init()

if not cyber.ok():
    print('Well, something is wrong.')
    sys.exit(1)

# Do your job here.
cyber.shutdown()
```

Learn more usage from the [examples](cyber_py3/examples/) and
[tests](cyber_py3/test/).

