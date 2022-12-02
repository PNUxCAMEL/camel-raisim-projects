# CAMEL-TOOLS
>This project is camel tools
- Inho Lee, [inholee8@pusan.ac.kr](inholee8@pusan.ac.kr)
- Hosun Kang, [hosun7379@pusan.ac.kr](hosun7379@pusan.ac.kr)
- Jeahoon Ahn, [dkswogns46@gmail.com](dkswogns46@gmail.com)
---
## How to Install
1. Remove 'example' in the CMakeList.txt
```cmake
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)
PROJECT(camel-tools)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(filter)
add_subdirectory(optimizer)
add_subdirectory(sensor)
add_subdirectory(thread)
add_subdirectory(trajectory)
#add_subdirectory(examples)
```
2. Build and install libaries(tools)
```text
cd camel-tools
mkdir build
cd build
cmake ..
make
sudo make install
```
3. (Optional) If you want to use example, add subdirectory in the CMakeList.txt
```cmake
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)
PROJECT(camel-tools)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(filter)
add_subdirectory(optimizer)
add_subdirectory(sensor)
add_subdirectory(thread)
add_subdirectory(trajectory)
add_subdirectory(examples)
```

## How to Use
- Write commands in your CMakeList.txt like below
```cmake
find_package(camel-tools-filter REQUIRED)
find_package(camel-tools-optimizer REQUIRED)
find_package(camel-tools-sensor REQUIRED)
find_package(camel-tools-thread REQUIRED)
find_package(camel-tools-trajectory REQUIRED)

target_link_libraries(<PROJECT_NAME> camel-tools-filter)
target_link_libraries(<PROJECT_NAME> camel-tools-optimizer)
target_link_libraries(<PROJECT_NAME> camel-tools-sensor)
target_link_libraries(<PROJECT_NAME> camel-tools-thread)
target_link_libraries(<PROJECT_NAME> camel-tools-trajectory)
```

## Dependencies
1. Build 'MSCL' to use Micro strain lord imu
```text
cd Downloads/
wget https://github.com/LORD-MicroStrain/MSCL/releases/latest/download/c++-mscl_65.0.0_amd64.deb
sudo dpkg -i c++-mscl_65.0.0_amd64.deb    
sudo apt install -f                 

# Edit udev rule
sudo gedit /etc/udev/rules.d/100-microstrain.rules

#add the following rules in the '100-microstrain.rules'
SUBSYSTEM=="tty", ATTRS{idVendor}=="199b", ATTRS{idProduct}=="3065", PROGRAM="/bin/sh -c 'echo %s{serial} | sed s/^0000__.*/rtk/g | sed s/^0000.*/main/g | sed s/^2-00.*/aux/g'", SYMLINK+="microstrain_%c", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", PROGRAM="/bin/sh -c 'echo %s{serial} | sed s/^0000__.*/rtk/g | sed s/^0000.*/main/g | sed s/^2-00.*/aux/g'", SYMLINK+="microstrain_%c", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="199b", ATTRS{idProduct}=="3065", PROGRAM="/bin/sh -c 'echo %s{serial} | sed s/^0000__/rtk/g | sed s/^0000/main/g | sed s/^2-00/aux/g'", SYMLINK+="microstrain_%c", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", PROGRAM="/bin/sh -c 'echo %s{serial} | sed s/^0000__/rtk/g | sed s/^0000/main/g | sed s/^2-00/aux/g'", SYMLINK+="microstrain_%c", MODE="0666"

sudo udevadm control --reload-rules && sudo systemctl restart systemd-udevd && sud
```

2. Build 'realsense' library to use T265
```text
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
sudo update-grub && sudo reboot
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
./scripts/patch-realsense-ubuntu-lts.sh
mkdir build && cd build
cmake ..
sudo make uninstall && make clean && make && sudo make install
realsense-viewer
```
