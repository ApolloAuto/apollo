编译步骤为：
1.	进入文件夹`modules/dreamview/frontend`
2.	执行指令 `yarn install`
3.	上一条指令会出错，错误信息为：`PhantomJS not found on PATH`
4.	执行脚本`frontend_adapter_for_TX2.sh`下载TX2适用的工具
5.	在当前文件夹执行执行`yarn build或者在`/apollo`文件夹下执行`./apollo.sh build_fe`编译前端
