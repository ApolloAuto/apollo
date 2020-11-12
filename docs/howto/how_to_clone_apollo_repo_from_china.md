# How to Clone Apollo Repository from China

This document describes a smarter approach to clone Apollo repository for users
from China.

## Pre-requisites

- Suppose you have forked Apollo Repository on GitHub. This is done by clicking
  the "Fork" button on the top-right of
  [Apollo's Github Page](https://github.com/ApolloAuto/apollo.git) and follow
  the instructions there.

## First, clone Apollo from Gitee

Apollo on Gitee was one day later than Apollo on GitHub. We can use it
as a good start.

```bash
git clone https://gitee.com/ApolloAuto/apollo.git
```

This step takes only serveral minutes, as the download rate can reach up-to >
10MiB/s tested on my machine with

the following console log:

```text
Cloning into 'apollo'...
remote: Enumerating objects: 313277, done.
remote: Counting objects: 100% (313277/313277), done.
remote: Compressing objects: 100% (66199/66199), done.
remote: Total 313277 (delta 245822), reused 310653 (delta 243198), pack-reused 0
Receiving objects: 100% (313277/313277), 2.19 GiB | 11.10 MiB/s, done.
Resolving deltas: 100% (245822/245822), done.
Checking out files: 100% (9124/9124), done.
```

## Reset origin to your GitHub fork

```bash
git remote set-url origin git@github.com:YOUR_GITHUB_USERNAME/apollo.git
```

## Set Apollo GitHub repo as upstream

```bash
# Using SSH
git remote add upstream git@github.com:ApolloAuto/apollo.git

# Using HTTPS
git remote add upstream https://github.com/ApolloAuto/apollo.git
```

You can confirm that the origin/upstream has been setup correctly by running:

```bash
git remote -v
```

It will show the list of remotes similar to the following:

```text
origin	git@github.com:YOUR_GITHUB_USERNAME/apollo.git (fetch)
origin	git@github.com:YOUR_GITHUB_USERNAME/apollo.git (push)
upstream	git@github.com:ApolloAuto/apollo.git (fetch)
upstream	git@github.com:ApolloAuto/apollo.git (push)
```

## Pull latest code from GitHub upstream

```bash
git pull --rebase upstream master
```

You can follow this guide on
[How to Workaround Slow Pull from CN](how_to_solve_slow_pull_from_cn.md).

## Congrats!

You have successfully cloned Apollo repository. Good luck to your journey of
autonomous driving with Apollo!
