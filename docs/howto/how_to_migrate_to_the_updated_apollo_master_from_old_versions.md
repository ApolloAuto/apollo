# Migrate to Apollo master from old versions

## Introduced Git-LFS

In Apollo master, we enabled [Git-LFS](https://git-lfs.github.com) as the storage
for large files, such as records, images, models, etc.

*To make the most out of Git-LFS, we also revised the whole history of Apollo to
move data files into the large-file-storage.*

This makes the Apollo master more like a brand new repo, while NOT growing on top
of Apollo 3.0, as they have different commit ID on almost each history commits.

```
Note:
- Please re-fork and re-clone your repository to sync with the latest changes.
- Rememeber to make a copy of your work (changes) before re-cloning or you will lose all of it.
```

### Why we are doing this

Besides code, Apollo also has a lot of data files which are generally very big.
The repo will increase quickly over time, and brings a big pain for everyone, as
you may have already encountered in pulling the 1GB+ Apollo code base.

So we decided to leverage [Git-LFS](https://git-lfs.github.com) plugin to manage
the data files. The plugin only retrives the exact version of file from server
when you are checking out a commit. As a result, it will save a lot of space and
time in the long run.

### What will be affected, and suggested actions

1. First of all, install it!

   The data files become simple pointers in the Git repo, which relies on the
   Git-LFS plugin to translate them to files with correct content. So, please
   follow the latest docs/quickstart/apollo_software_installation_guide.md to
   install all required software stack.

   As mentioned in the guide, newer version of Git has very good lfs-integration
   and works transparently, while for old version, like the one in Ubuntu 14.04,
   you may have to trigger it explicitly:

   ```bash
      git lfs clone <args>  # Instead of git clone
      git lfs pull <args>   # Instead of git pull
   ```

   In the following parts, we will assume that you are using newer Git.

1. Try your best to understand how it works.

   If it's setup correctly, you don't need to know about it for daily work. But
   it's still strongly recommended to understand how it works.

   Please checkout the official
   [tutorial](https://github.com/git-lfs/git-lfs/wiki/Tutorial) for more
   information.

1. Checkout, fetch, pull or hard reset to a commit.

   If you have installed git-lfs correctly, these actions would be very simple.
   Just do it as usual.

1. Rebase your work. *Please don't do this!*

   As we have revised the history, the ID of the base commit of your work
   actually changed. Instead of rebasing your work to a commit, please do it in
   a reversed way:

   - Checkout the target commit: `git checkout <target_commit>`
   - Cherry-pick your work:
     `git cherry-pick <base_commit_of_your_work>...<HEAD_of_your_work>`

1. Migrate your work to the same base commit in Apollo master.

   As we have revised the history, the ID of the base commit of your work
   actually changed. Please find the equivalent commit in Apollo master:

   ```bash
   git log --pretty=oneline | grep "<commit meassge>"
   ```

   Then follow the steps above to "Rebase your work" to that commit.

1. Update your fork.

   As we have revised the history, you have to do a forced update of your repo,
   which is like `git push -f origin master`. If you just need a clean fork of
   Apollo, it's highly recommended to delete your old fork and fork again from
   the upstream repo and then clone it to your local.

### Trouble shooting guide

1. Missed file found when run `git push -f origin master`. If you see


   ```
   open /home/apollo/docs/specs/Camera/images/Argus_Specs1.png: no such file or directory        
   ```

   please run `git lfs fetch --all` before git push to resolve the issue.
