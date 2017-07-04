## How to Add a New External Dependency
A design and implementation goal is to minimize the dependency that must be pre-installed in the system. If your target depends on a module for which you have to `apt-get install` first, consider using bazel as the package/dependency management system.

For example if you want to add a workspace rule `foo` that is not originally built with bazel, do the following:

- Add a workspace rule named 'foo' in the WORKSPACE file.
- Specify the source of `foo` (usually a URL), and the version (usually a commit hash or a git tag).
- Write a `foo.BUILD` under a third_party directory to build it. The BUILD file will be similar to any other bazel BUILD file of your own targets.
- In your target that depends on `foo`, put `@foo://:<foo_target>` in its dependencies.

### Use Bazel to Add an External Dependency

If you add a workspace rule `foo`  using bazel to build your target, depending on `foo`, bazel pulls the source code of `foo` from the source specified, and builds it with `foo.BUILD`. If `foo` was originally built with bazel, then only the workspace rule is needed.

Click on the following links for a more detailed description on adding a dependency with bazel:
[Workspace Rules](https://bazel.build/versions/master/docs/be/workspace.html),
[Working with external dependencies](Working with external dependencies).
