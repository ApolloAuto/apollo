# How to Create a Pull Request

This document is a brief step-by-step guide on creating pull requests for
Apollo. Your can also refer to
[GitHub: Using Pull Requests](https://help.github.com/articles/using-pull-requests/)
for a thorough understanding.

## Step 1: Fork your own copy of ApolloAuto/apollo to your GitHub account

This is done by clicking the "Fork" button on the top-right of
[Apollo's Github Page](https://github.com/ApolloAuto/apollo) and following the
guide there.

## Step 2: Clone your fork of the repo

Note:

> Please replace "YOUR_USERNAME" with your GitHub account in the descriptions
> below.

Open a terminal, type either of the following commands:

```
# Using SSH
git clone git@github.com:YOUR_USERNAME/apollo.git

# Using HTTPS
git clone https://github.com/YOUR_USERNAME/apollo.git
```

## Step 3: Set up your username and email for this repo

```
git config user.name  "My Name"
git config user.email "myname@example.com"
```

## Step 4: Set official Apollo repo as upstream

Configuring an upstream remote allows you to sync changes made in the upstream
with your own fork.

This is done with the following command:

```
# Using SSH
git remote add upstream git@github.com:ApolloAuto/apollo.git

# Using HTTPS
git remote add upstream https://github.com/ApolloAuto/apollo.git
```

You can confirm that the upstream repo has been added by running:

```
git remote -v
```

If successful, it will show the list of remotes similar to the following:

```
origin	git@github.com:YOUR_USERNAME/apollo.git (fetch)
origin	git@github.com:YOUR_USERNAME/apollo.git (push)
upstream	git@github.com:ApolloAuto/apollo.git (fetch)
upstream	git@github.com:ApolloAuto/apollo.git (push)
```

## Step 5: Create a new branch; Make and commit changes

```
git checkout -b my_dev origin/master

# Make your own changes on branch "my_dev"
# blah blah ...

# Commit to your own branch with commit msg:
git commit -m "[module] brief description of the changes"
```

## Step 6: Sync up with upstream ApolloAuto/apollo

```
git pull --rebase  upstream master
```

## Step 7: Push your local changes to your fork.

```
git push -f -u origin my_dev
```

## Step 8: Generate a pull request

Create a new pull request between "Apolloauto/apollo:master" and
"YOUR_USERNAME/apollo:my_dev" by clicking the "Pull Request" button on
[your forked Apollo repo page](https://github.com/YOUR_USERNAME/apollo) on
GitHub.

You can then follow the steps described by
[GitHub: Creating a Pull Request](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/creating-a-pull-request-from-a-fork)
on what to do next.

Note:

> Please don't forget to **add the description of your PR**. It can help
> reviewers better understand the changes you have made and the intention for
> those changes.

Collaborators from our team will be glad to review and merge your commit! (This
may take some time, please be patient.)

## Step 9: Done!

Thanks a lot for your PR!
