# 国内环境 GitHub 拉取仓库速度慢的缓解方案

## 第一步: 浏览器打开如下两个网址，找到对应 IP 地址:

1. http://github.com.ipaddress.com/
2. http://github.global.ssl.fastly.net.ipaddress.com/

假设对应 IP 地址分别为 140.82.xx.xxx 和 199.232.yy.yyy

## 第二步: 编辑 hosts 文件

```
sudo vim /etc/hosts
```

加入如下两行:

```
140.82.xx.xxx  github.com
199.232.yy.yyy github.global.ssl.fastly.net
```

## 第三步：更新 DNS 缓存（仅限 Mac 用户）

Linux 用户请忽略此步骤。

```
sudo dscacheutil -flushcache
```

大功告成!

现在，您可以尝试从 GitHub 重新拉取代码了，是不是变快了一些？

