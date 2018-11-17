# How to Install Ubuntu Linux Operating System

## Helpful hints before you begin:

* It is recommended that you use Ubuntu 14.04.3.
* You can type F2 during the system boot process to enter the BIOS settings. It is recommended that you disable Quick Boot and Quiet Boot in the BIOS to make it easier to catch any issues in the boot process.
* The IPC must have internet access to update and install software. Make sure that the Ethernet cable is connected to a network with Internet access. You might need to configure the network for the IPC if the network that it is connected to is not using the Dynamic Host Configuration Protocol (DHCP).
* For more information about Ubuntu, see Ubuntu for Desktop web site: https://www.ubuntu.com/desktop.
## Installing Ubuntu Linux:
Follow these steps:
1.	Create a bootable Ubuntu Linux USB flash drive:
Download Ubuntu (or a variant such as Xubuntu) and follow the online instructions to create a bootable USB flash drive.
2.	Install Ubuntu Linux:
    * Insert the Ubuntu installation drive into a USB port and turn on the system. 
    * Install Linux by following the on-screen instructions.
3.	Perform a software update and installation: 
    * Reboot into Linux after the installation is done. 
    * Launch the Software Updater to update to the latest software packages (for the installed distribution) or type the following commands in a terminal program such as GNOME Terminal.
      ```sudo apt-get update``` |
      ```sudo apt-get upgrade```
4.  Launch a terminal program such as GNOME Terminal and type the following command to install the Linux 4.4 kernel:
      ```sudo apt-get install linux-generic-lts-xenial```
