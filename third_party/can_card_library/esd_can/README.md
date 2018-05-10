## ESD CAN Library

In the first release of Apollo software, ESD PCIe CAN is supported out of the box -- please refer to *Apollo 1.0 Hardware and System Installation Guide* for more information. However, you MUST obtain ESD's CAN library software from ESD Electronics to compile and use it in Apollo software stack.

1. After/when you purchase CAN card from ESD Electronics, please contact support@esd.eu to obtain the supporting software package for Linux.
  The CAN card to use with the IPC is ESD CAN-PCIe/402. For more information about the CAN-PCIe/402, see the [ESD CAN-PCIe/402 Product Page](https://esd.eu/en/products/can-pcie402)

2. After unpacking the software package, please find and copy the following files to the specific sub-directories (under this directory):
  * Copy ntcan.h to include/
  * Copy 64-bit libntcan.so.4.0.1 to lib/
  * Do the following to add the necessary symbolic links:

        cd ./lib/;
        ln -s libntcan.so.4.0.1 libntcan.so.4;
        ln -s libntcan.so.4.0.1 libntcan.so.4.0
