# 如何安装linux操作系统

## 开始工作前的帮助性信息

* 我们推荐使用Ubuntu 14.04。

* 在系统启动时按下F2（或其他按键，请参考系统文档）进入BIOS设置，我们建议禁用Quick Boot和Quiet Boot设置以更容易的在启动时捕获错误信息。

* IPC必须有网络连接以更新和安装软件。确保IPC的以太网线接入了有互联网访问权限的网络。如果接入的网络没有使用动态主机配置协议（DHCP），使用者可能需要对IPC的网络进行配置。

* 更多关于Ubuntu的信息请参考Ubuntu桌面版的网站：https://www.ubuntu.com/desktop。

## 安装Ubuntu Linux
按照下述步骤执行：
1.	创建一个可引导的Ubuntu Linux USB启动盘：

    下载Ubuntu 14.04（或其他的变种系统如Xubuntu）并创建一个可引导的USB启动盘。

2.	安装Ubuntu Linux：

    * 将安装Unbuntu的USB启动盘插入USB接口中并启动系统
    * 按照屏幕提示执行安装
    
3.	执行软件更新：

    * 安装结束后重启并进入系统
    * 启动Software Update并更新到最新软件包，或在终端程序如GNOME Terminal中执行下述指令完成更新：
      ```sudo apt-get update``` |
      ```sudo apt-get upgrade```
      
4.  启动终端程序如GNOME Terminal，执行下述指令安装Linux 4.4内核：

      ```sudo apt-get install linux-generic-lts-xenial```
