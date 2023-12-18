# Build configurations for TensorRT.

def if_tensorrt(if_true, if_false=[]):
  """Tests whether TensorRT was enabled during the configure process."""
  return %{if_tensorrt}


def if_tensorrt_version_8(if_true, if_false=[]):
    """Read the tensorrt version from env variable.
    Returns a select statement which evaluates to if_true if version of tensorrt
    Greater than or equal to 8. Otherwise, the select statement evaluates to
    if_false.
    """
    _TF_VERSION = "%{_TF_TENSORRT_VERSION}"
    _TF_MAJOR_VERSION = int(_TF_VERSION.split(".")[0])
    if _TF_MAJOR_VERSION >= 8:
      return if_true

    return if_false 