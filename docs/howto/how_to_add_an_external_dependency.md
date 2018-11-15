## How to Add a New External Dependency

The design and implementation goal is to minimize the dependency that must be pre-installed in the system. If your target depends on a module for which you have to `apt-get install` first, consider using **Bazel** as the package/dependency management system.

For example, if you want to add a workspace rule `foo` that is not originally built with Bazel, do the following:

- Add a workspace rule named 'foo' in the WORKSPACE file.
- Specify the source of `foo` (usually a URL), and the version (usually a commit hash or a git tag).
- Write a `foo.BUILD` under the third_party directory to build it. The BUILD file will be similar to any other Bazel BUILD file of your own targets.
- In your target that depends on `foo`, put `@foo://:<foo_target>` in its dependencies.

### Use Bazel to Add an External Dependency

If you add a workspace rule `foo`  using Bazel to build your target, depending on `foo`, Bazel pulls the source code of `foo` from the source specified, and builds it with `foo.BUILD`. If `foo` was originally built with Bazel, then only the workspace rule is needed.

### References

For a detailed description on adding a dependency with Bazel, refer to the following:
* [Workspace Rules](https://bazel.build/versions/master/docs/be/workspace.html)
* [Working with external dependencies](https://docs.bazel.build/versions/master/external.html).
