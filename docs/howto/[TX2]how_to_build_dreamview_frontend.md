Follow these steps:
1.	enter directory `modules/dreamview/frontend`
2.	execute cmmand `yarn install`
3.	the last command does not work, an error like this occurs: `PhantomJS not found on PATH`
4.	execute script `frontend_adapter_for_TX2.sh` to download tools for TX2
5.	execute command `yarn build` in current directory or enter directory `/apollo` and execute command `./apollo.sh build_fe` to build dreamview frontend.
