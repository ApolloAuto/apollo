# 如何创建合入请求(Pull Request)

本文档以 Apollo 项目为例，引导新手逐步熟悉创建代码合入请求（Pull Request）的步骤
。另，可参考[GitHub 页](https://help.github.com/articles/using-pull-requests/)
了解更多。

## 第一步：创建您个人的 Apollo 代码分支

这可以通过点击[Apollo 的 GitHub 页](https://github.com/ApolloAuto/apollo) 右上角
的【Fork】按钮并按照其指引操作来完成。

## 第二步：克隆您的 Apollo 分支仓库

注：

> 请用您的 GitHub 用户名替换以下描述中的"YOUR_USERNAME"。

打开终端，输入以下任一命令：

```
# 使用 SSH 方式
git clone git@github.com:YOUR_USERNAME/apollo.git

# 使用 HTTPS 方式
git clone https://github.com/YOUR_USERNAME/apollo.git
```

## 第三步：设置您的用户名和电子邮件地址

```
git config user.name  "My Name"
git config user.email "myname@example.com"
```

## 第四步： 将 Apollo 官方仓库设为 upstream 上游分支

设置上游分支以便后续同步远端 upstream 分支的代码变更，这可以通过如下命令完成：

```
# 使用SSH
git remote add upstream git@github.com:ApolloAuto/apollo.git

# 使用 HTTPS
git remote add upstream https://github.com/ApolloAuto/apollo.git
```

通过如下命令查看 upstream 是否设置成功：

```
git remote -v
```

如成功，则会显示如下的 remotes 列表：

```
origin	git@github.com:YOUR_USERNAME/apollo.git (fetch)
origin	git@github.com:YOUR_USERNAME/apollo.git (push)
upstream	git@github.com:ApolloAuto/apollo.git (fetch)
upstream	git@github.com:ApolloAuto/apollo.git (push)
```

## 第五步：创建分支，做出修改，提交变更

```
git checkout -b my_dev origin/master

# 在您的my_dev分支修复问题，添加新功能，等等
# ...

# 将代码变动提交到您的本地分支，注意提交消息格式
git commit -m "[module] brief description of the changes"
```

## 第六步：同步上游仓库变更

```
git pull --rebase  upstream master
```

## 第七步：将您的本地修改推送到您个人的 Apollo 分支仓库

```
git push -f -u origin my_dev
```

## 第八步，生成代码合并请求

通过点击您的 Apollo 克隆 GitHub 页（通常
为https://github.com/YOUR_USERNAME/apollo) 上的"Pull Request" 按钮新建从
"YOUR_USERNAME/apollo:my_dev" 到 "Apolloauto/apollo:master" 的代码合并请求。

可参
考[GitHub 页：创建合并请求](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/creating-a-pull-request-from-a-fork)
的描述来完成代码合并请求。

**注**：

> 请不要忘了添加您的 PR 的描述。PR 描述可以帮助代码评审人员更好地理解您的代码变
> 更的意图。

作为开源项目，我们作为 Apollo 团队成员，很乐意评审并合入您提交的代码。但限于精力
，可能您的 PR 从提交到合入需要花一些时间，请给我们一点耐心。

## 最后：大功告成

感谢您的 PR！
