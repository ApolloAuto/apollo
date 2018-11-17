# How to create a pull request

You can follow the standard [github approach](https://help.github.com/articles/using-pull-requests/) to contribute code to Apollo. Here is a sample setup:

- Fork a new repo with your GitHub username.
- Set up your GitHub personal email and user name

```
git config user.name "XXX"
git config user.email "XXX@[XXX.com]"
```

- Clone your fork (Please replace "USERNAME" with your GitHub user name.)

```
(Use SSH) git clone git@github.com:USERNAME/apollo.git
(Use HTTPS) git clone https://github.com/USERNAME/apollo.git
```

- Add Apollo repository as upstream

```
(Use SSH) git remote add upstream git@github.com:ApolloAuto/apollo.git
(Use HTTPS) git remote add upstream https://github.com/ApolloAuto/apollo.git
```

- Confirm that the upstream branch has been added
```
git remote -v
```

- Create a new branch, make changes and commit

```
git checkout -b  "my_dev"
```

- Sync up with the Apolloauto/apollo repo

```
git pull --rebase  upstream master
```

- Push local developments to your own forked repository

```
git push -f -u origin "my_dev"
```

- Generate a new pull request between "Apolloauto/apollo:master" and "forked repo:my_dev"
- Collaborators will review and merge the commit (this may take some time, please be patient)

Thanks a lot for your contributions!
