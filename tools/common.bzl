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

def _path_is_absolute(path):
    """Returns `True` if `path` is an absolute path.

    Args:
      path: A path (which is a string).

    Returns:
      `True` if `path` is an absolute path.
    """
    return path.startswith("/") or (len(path) > 2 and path[1] == ":")

def join_paths(path, *others):
    """Joins one or more path components intelligently.

    This function mimics the behavior of Python's `os.path.join` function on POSIX
    platform. It returns the concatenation of `path` and any members of `others`,
    inserting directory separators before each component except the first. The
    separator is not inserted if the path up until that point is either empty or
    already ends in a separator.

    If any component is an absolute path, all previous components are discarded.

    Args:
      path: A path segment.
      *others: Additional path segments.

    Returns:
      A string containing the joined paths.
    """
    result = path

    for p in others:
        if _path_is_absolute(p):
            result = p
        elif not result or result.endswith("/"):
            result += p
        else:
            result += "/" + p

    return result

## Adapted from RobotLocomotion/drake:tools/skylark/pathutils.bzl
# Remove prefix from path.
def ___remove_prefix(path, prefix):
    # If the prefix has more parts than the path, failure is certain.
    if len(prefix) > len(path):
        return None

    # Iterate over components to determine if a match exists.
    for n in range(len(prefix)):
        if prefix[n] == path[n]:
            continue
        elif prefix[n] == "*":
            continue
        else:
            return None

    return "/".join(path[len(prefix):])

def __remove_prefix(path, prefix):
    # Ignore trailing empty element (happens if prefix string ends with "/").
    if len(prefix[-1]) == 0:
        prefix = prefix[:-1]

    # If the prefix has more parts than the path, failure is certain. (We also
    # need at least one component of the path left over so the stripped path is
    # not empty.)
    if len(prefix) > (len(path) - 1):
        return None

    # Iterate over components to determine if a match exists.
    for n in range(len(prefix)):
        # Same path components match.
        if prefix[n] == path[n]:
            continue

        # Single-glob matches any (one) path component.
        if prefix[n] == "*":
            continue

        # Mulit-glob matches one or more components.
        if prefix[n] == "**":
            # If multi-glob is at the end of the prefix, return the last path
            # component.
            if n + 1 == len(prefix):
                return path[-1]

            # Otherwise, the most components the multi-glob can match is the
            # remaining components (len(prefix) - n - 1; the 1 is the current
            # prefix component) less one (since we need to keep at least one
            # component of the path).
            k = len(path) - (len(prefix) - n - 1)

            # Try to complete the match, iterating (backwards) over the number
            # of components that the multi-glob might match.
            for t in reversed(range(n, k)):
                x = ___remove_prefix(path[t:], prefix[n + 1:])
                if x != None:
                    return x

            # Multi-glob failed to match.
            return None

        # Components did not match.
        return None

    return "/".join(path[len(prefix):])

def remove_prefix(path, prefix):
    """Remove prefix from path.

    This attempts to remove the specified prefix from the specified path. The
    prefix may contain the globs ``*`` or ``**``, which match one or many
    path components, respectively. Matching is greedy. Globs may only be
    matched against complete path components (e.g. ``a/*/`` is okay, but
    ``a*/`` is not treated as a glob and will be matched literally). Due to
    Starlark limitations, at most one ``**`` may be matched.

    Args:
        path (:obj:`str`) The path to modify.
        prefix (:obj:`str`) The prefix to remove.

    Returns:
        :obj:`str`: The path with the prefix removed if successful, or None if
        the prefix does not match the path.
    """
    return __remove_prefix(path.split("/"), prefix.split("/"))

def output_path(ctx, input_file, strip_prefix, package_root = None):
    """Compute "output path".

    This computes the adjusted output path for an input file. Specifically, it
    a) determines the path relative to the invoking context (which is usually,
    but not always, the same as the path as specified by the user when the file
    was mentioned in a rule), without Bazel's various possible extras, and b)
    optionally removes prefixes from this path. When removing prefixes, the
    first matching prefix is removed.

    This is used primarily to compute the output install path, without the
    leading install prefix, for install actions.

    For example::

        install_files(
            dest = "docs",
            files = ["foo/bar.txt"],
            strip_prefix = ["foo/"],
            ...)

    The :obj:`File`'s path components will have various Bazel bits added. Our
    first step is to recover the input path, ``foo/bar.txt``. Then we remove
    the prefix ``foo``, giving a path of ``bar.txt``, which will become
    ``docs/bar.txt`` when the install destination is added.

    The input file must belong to the current package; otherwise, ``None`` is
    returned.

    Args:
        input_file (:obj:`File`): Artifact to be installed.
        strip_prefix (:obj:`list` of :obj:`str`): List of prefixes to strip
            from the input path before prepending the destination.

    Returns:
        :obj:`str`: The install destination path for the file.
    """

    if package_root == None:
        # Determine base path of invoking context.
        package_root = join_paths(ctx.label.workspace_root, ctx.label.package)

    # Determine effective path by removing path of invoking context and any
    # Bazel output-files path.
    input_path = input_file.path
    if input_file.is_source:
        input_path = remove_prefix(input_path, package_root)
    else:
        out_root = join_paths("bazel-out/*/*", package_root)
        input_path = remove_prefix(input_path, out_root)

    # Deal with possible case of file outside the package root.
    if input_path == None:
        return None

    # Possibly remove prefixes.
    for p in strip_prefix:
        output_path = remove_prefix(input_path, p)
        if output_path != None:
            return output_path

    return input_path
