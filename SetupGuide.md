# Soft Origami Arm SD card Setup Procedure

This is a guide to build the SD card for the Soft Origami Arm from zero.

# Ubuntu_18.04 SD card preparation
Download Ubuntu 18.04 from
http://cdimage.ubuntu.com/releases/bionic/release/ubuntu-18.04.4-preinstalled-server-arm64+raspi3.img.xz

Make the SD card, using https://www.raspberrypi.org/downloads/
## Network configuration 
To get internet access, modify the netplan file
```
>>sudo nano /etc/netplan/50-cloud-init.yaml
```
to
```
network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
    version: 2
    wifis:
        wlan0:
            access-points:
                "nameOfSSID":
                     password: "1234"
            dhcp4: true
            optional: true
```

Then apply the netplan by
```
>>sudo netplan apply
```

## Install the desktop

```
>>sudo apt update
>>sudo apt upgrade
>>sudo apt install xubuntu-desktop
```


## install Python3 related package
```
>>sudo apt install python3-pip
>>pip3 install catkin-tools
>>pip3 install rospkg
>>pip3 install cython --upgrade 
>>pip3 install numpy --upgrade
>>pip3 install angles
>>sudo apt-get install libatlas-base-dev gfortran
>>sudo pip3 install scipy
```


## ROS installation & catkin workspace creation

refer to [http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)

## configure joystick in ROS

refer to [http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)


# Get origarm_ros package
```
cd ~/catkin_ws/src
git clone https://github.com/XiaojiaoChen/origarm_ros

```

## Enable spi & permissions 
```
>>lsmod | grep -i spi
>>ls -l /dev/spi*                
>>cd /etc/udev/rules.d/
>>sudo nano local.rules
```
Add the following to the local.rules
```
ACTION=="add", KERNEL=="spidev0.0", MODE="0666"
```

## Enable keyboard
install evtest tool
```
>>sudo apt install evtest -y
```
run evtest and check which event (e.g. eventX) corresponds to keyboard
```
>>sudo evtest
```
check path link for keyboard and copy the link to defualt_path[] in origarm_ros/src/keyboard.cpp
```
>>ls -h /dev/input/by-path/
```
check the permission for keyboard
```
>>ls -l /dev/input/eventX
```
configure rw permission for keyboard
```
>>sudo chmod a+rw /dev/input/eventX
```

## 7-inch external screen (optional)
Open SD card from card reader, in **system_boot**, add to **usrconig.txt** with
```
max_usb_current=1
hdmi_group=2
hdmi_mode=87
hdmi_cvt 800 480 60 6 0 0 0
hdmi_drive=1
``` 


## Clone a SD

prepare another SD card through usb, check by

```
lsblk -f 
```
Normally, mmcblk0 is the original system SD card，sda is the new SD card.
```
sudo dd bs=4M if=/dev/mmcblk0 of=/dev/sda status=progress
```

## install Eigen library

```
>>sudo apt-get install libeigen3-dev
```
configure library (defualt library for installation: /usr/include/)
```
>>cd /usr/include/
>>sudo ln -sf eigen3/Eigen Eigen
>>sudo ln -sf eigen3/unsupported unsupported
```

configure CMakeList.txt
```
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories($EIGEN3_INCLUDE_DIR)
add_definitions(${EIGEN_DEFINITIONS})
```