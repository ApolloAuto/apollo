# Python Wrapper for Cyber RT

## Usage

Make sure you have built Apollo successfully, which should also have added
`/apollo/cyber/python` to the PYTHONPATH for you. Then in Python code:

```python
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

## Work with Python2?

Firstly, it's not recommended, as Python2 is deprecated since
[Jan 1, 2020](https://pythonclock.org). We also deprioritized maintaining the
Python2 Wrapper.

Similar to the Python 3 wrapper, but just import things from the cyber_py
module. Everything should work the same.

```python
import sys

from cyber_py import cyber


cyber.init()

if not cyber.ok():
    print('Well, something is wrong.')
    sys.exit(1)

# Do your job here.
cyber.shutdown()
```

Learn more usage from the [examples](cyber_py/examples/) and
[tests](cyber_py/test/).
