# Github访问慢解决方案

浏览器打开如下网站

http://github.global.ssl.fastly.net.ipaddress.com/

找到对应IP地址，例如：151.101.xx.xx



浏览器打开另外一个网站

http://github.com.ipaddress.com/

找到对应IP地址。例如：192.30.xx.xx



编辑hosts文件

```
sudo vim /etc/hosts
```

在文件中加入如下两行

```
192.30.xx.xx github.com
151.101.xx.xx github.global.ssl.fastly.net
```



如果使用mac，还需更新DNS缓存

```
sudo dscacheutil -flushcache
```

