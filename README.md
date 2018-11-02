# COOLEYE-D1 WIN10 SDK 

因为WIN已经宣布支持ROS，因此本相机的WIN驱动也会逐步更新，满足WIN环境下用户的使用需求。
项目发布地址：
https://github.com/cooleyecam/cooleye_d1_windows

### 1.SDK开发环境安装

- VS2017
- OPENCV 3.4.0
- QT 5.10.1
- libusb1.0

目前SDK对OPENCV和QT的依赖主要在于图像显示和界面，所用函数和软件版本耦合度并不高，但随着后续功能的更新可能会逐步加深耦合度，请尽量使用以上的软件版本以避免出现不必要的错误，高级用户可根据自己需求切换软件版本。

- 进入VS后，文件-->打开-->项目/解决方案
- 生成-->清理解决方案
- 生成-->-->清理CoolEye
皆可成功编译项目并生成文件。


### 2.USB驱动安装
Cooleye-D1设备在win10下使用libusb读取设备数据，可只用zadig-2.3直接安装。
- 勾选 Zadig => Options => List All Devices 
- 下拉菜单选择 CoolEye-D1
- Driver选择 WinUSB(v6.1.7600.16385)
- 点击Replace Driver
- 重新拔插设备即可




 
