# Sanitize a dependency so that it works correctly from code that includes
# TensorFlow as a submodule.
def clean_dep(dep):
    return str(Label(dep))

# Ref: bazel-skylib@lib/paths.bzl
def basename(p):
    """Returns the basename (i.e., the file portion) of a path.
    Note that if `p` ends with a slash, this function returns an empty string.
    This matches the behavior of Python's `os.path.basename`, but differs from
    the Unix `basename` command (which would return the path segment preceding
    the final slash).
    Args:
      p: The path whose basename should be returned.
    Returns:
      The basename of the path, which includes the extension.
    """
    return p.rpartition("/")[-1]

def dirname(p):
    """Returns the dirname of a path.
    The dirname is the portion of `p` up to but not including the file portion
    (i.e., the basename). Any slashes immediately preceding the basename are not
    included, unless omitting them would make the dirname empty.
    Args:
      p: The path whose dirname should be returned.
    Returns:
      The dirname of the path.
    """
    prefix, sep, _ = p.rpartition("/")
    if not prefix:
        return sep
    else:
        # If there are multiple consecutive slashes, strip them all out as Python's
        # os.path.dirname does.
        return prefix.rstrip("/")
