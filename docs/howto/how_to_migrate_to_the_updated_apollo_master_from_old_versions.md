# Migrate to Apollo master from old versions

## Introduction

Due to a fatal bug with Git LFS that has caused restricted access to Apollo repos, we have decided to retire the service from all Apollo repos. On May 14th 2:07 PM Pacific Time, the Apollo Team has completed the migration. We are sorry for any inconveniences this may cause you.
``` 
Note:
If this is the first time you are cloning/building Apollo, you do not need to follow this guide. This guide is for people who had installed Git LFS previously with Apollo.
```
If this is your first time installing Apollo, please return to the [README](https://github.com/ApolloAuto/apollo/blob/master/README.md) page.

## Why did we retire Git LFS

Several developers complained about bugs in the build process as well as loss of access to our repo due to Git LFS

## Migration steps

We have compiled a list of steps that you can take to avoid being affected by this change. Please note, check if you have or have not made any local changes and/or submitted those changes to the old repo before starting the process:

### Not made any local changes

1. Disable LFS on your computer:
```
git lfs uninstall
```

2. Sync with our most up-to-date Apollo repo:

```bash
git pull --rebase upstream master #where “upstream” is your defined alias of our Apollo repo
```

3. Hard reset your forked repo:
``` 
git push -f original master
```

###  Made changes and have not submitted to the repo

1. Save your changes
2. Repeat the steps (1 - 3) in the section above
3. Re-apply the changes


### Made local changes and have submitted to your forked repo,
please cherry-pick those changes to the new repo and submit your commits.

```
Note:
If your repo did not sync with ours, you will still be using the Git LFS service. However, once the service is disabled, there is a high likelihood that your access to Apollo repos will be blocked/denied. To avoid such an incident, please follow the steps listed above that best 
fit your situation.
```

## Troubleshooting steps 

If you are still experiencing issues, you can always re-fork the repo. Let us know if you need any assistance on this process by creating an issue. 
