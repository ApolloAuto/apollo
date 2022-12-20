# 国内环境下如何克隆 Apollo 仓库

本文档描述了在国内环境下既快又好地克隆Apollo 仓库的方法。

## 必要前提/准备

- 假设你已经 「fork」 了GitHub上的Apollo 代码库。这可以通过点击[GitHub上的 Apollo](https://github.com/ApolloAuto/apollo.git) 页面右上角的「Fork」按钮并遵照随后的提示来完成。

##  首先，从码云上克隆 Apollo 仓库

[码云上的 Apollo仓库](https://gitee.com/ApolloAuto/apollo.git) 通常比[GitHub上的 Apollo仓库](https://github.com/ApolloAuto/apollo.git) 更新晚一天。

我们可以以它为起点，克隆码云上的Apollo 仓库。

执行命令：

```bash
git clone https://gitee.com/ApolloAuto/apollo.git
```

这一步通常很快，只需十数分钟。在本文作者所在的百度内部，下载速度可达 10 多 MiB/s，终端输出如下：

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

**注意：** 以下步骤可选。

##  重置远程分支origin

上述步骤完成后，就可以「过河拆桥」，将远程分支 origin 重置为你刚 「fork」的GitHub 分支。

```bash
git remote set-url origin git@github.com:<你的GitHub 用户名>/apollo.git
```

## 将 GitHub 上的 Apollo 仓库设为远程分支upstream

```bash
# 采用SSH的方式
git remote add upstream git@github.com:ApolloAuto/apollo.git

# 采用HTTPS的方式
git remote add upstream https://github.com/ApolloAuto/apollo.git
```

你可以运行如下命令来确认远程分支 origin 和 upstream 已被正确设置：

```bash
git remote -v
```

如果之前的操作正确，它将列出如下的远端分支：

```text
origin	git@github.com:<你的GitHub 用户名>/apollo.git (fetch)
origin	git@github.com:<你的GitHub 用户名>/apollo.git (push)
upstream	git@github.com:ApolloAuto/apollo.git (fetch)
upstream	git@github.com:ApolloAuto/apollo.git (push)
```

## 现在，从 GitHub 远程分支upstream 拉取最新的「增量」代码
 
```bash
git pull --rebase upstream master
```

如果遇到问题，你可以参考这篇文档：[国内环境拉取GitHub仓库慢的缓解方案](./how_to_solve_slow_pull_from_cn.md).

## 结语

恭喜你成功地克隆了 Apollo Gitee/GitHub 仓库，并由此开启你的 Apollo 自动驾驶之旅！

