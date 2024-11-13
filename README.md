

# eRob ROS2 MoveIt
---
# Installation Required Dependencies

---
## 1. Installing the Preempt-RT Real-Time Patch on Ubuntu 22.04

### 1.1 Check Kernel Version and Install Necessary Packages

```bash
uname -a
```
<div class="MoveIt">
  <a>
    <img src="/img/RT1.png" alt="RT1" style={{ width: 'auto', height: '500' }} />
  </a>
</div>


### 1.2  Install Necessary Packages

```bash
apt install autoconf automake libtool make libncurses-dev flex bison libelf-dev libssl-dev zstd net-tools
```
### 1.3 Download the Kernel and Patches

```bash
https://mirrors.edge.kernel.org/pub/linux/kernel/
```

Download the Linux kernel from the [Link](https://mirrors.edge.kernel.org/pub/linux/kernel/). Find your version—it's not the small files above, scroll down to find the Linux kernel files starting with "Linux" that are over 100MB.

<div class="MoveIt">
  <a>
    <img src="/img/RT2.png" alt="RT2" style={{ width: '1200', height: 'auto' }} />
  </a>
</div>

```bash
https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/
```
Download **the real-time patches** from the [Link](https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/). Make sure to choose the patch that matches your kernel version.

### 1.4 Extract and Apply the Patch

```bash
tar -zxvf linux-5.19.tar.gz
xz -d patch-5.19-rt10.patch.xz
```

```bash
cd linux-5.19/
patch -p1 < ../patch-5.19-rt10.patch
```

### 1.5  Kernel Configuration

```bash
make menuconfig
```

After entering the graphical configuration interface, perform the following operations:

- General Setup -> Preemption Model -> Fully Preemptible Kernel(RT)
- General Setup -> Timers subsystem -> Timer tick handling -> Full dynticks system
- General Setup -> Timers subsystem -> High Resolution Timer Support
- Processor type and features -> Timer frequency -> 1000 HZ

**Remember to save and exit.**

```bash
vi .config
```

``` txt
CONFIG_SYSTEM_TRUSTED_KEYS=""

CONFIG_SYSTEM_REVOCATION_KEYS=""
```
<div class="MoveIt">
  <a>
    <img src="/img/RT3.png" alt="RT3" style={{ width: '600', height: 'auto' }} />
  </a>
</div>
**Save and exit.**


### 1.6 Compile and Install

```bash
make -j8`nproc`
```
After completion:

```bash
make modules_install
make install
```

### 1.7 Configure GRUB Boot Options

```bash
vim /etc/default/grub
```
- Comment out the following line to display the boot menu:

```bash
GRUB_TIMEOUT_STYLE=hidden
```

- Adjust the timeout value:

```bash
GRUB_TIMEOUT=5  # Timeout in seconds
```

- Update GRUB configuration:

```bash 
update-grub
```
### 1.8 Reboot

<div class="MoveIt">
  <a>
    <img src="/img/RT5.png" alt="RT5" style={{ width: '600', height: 'auto' }} />
  </a>
</div>

<div class="MoveIt">
  <a>
    <img src="/img/RT6.png" alt="RT6" style={{ width: '600', height: 'auto' }} />
  </a>
</div>

<div class="MoveIt">
  <a>
    <img src="/img/RT7.png" alt="RT7" style={{ width: '600', height: 'auto' }} />
  </a>
</div>

### 1.9 Testing

```bash 
apt-get install rt-tests 
cyclictest -t 5 -p 80 -i 1000
```

`cyclictest` will perform 1000 cyclic tests over 5 seconds at the highest priority to measure the real-time performance of the Linux system. After the test, cyclictest will output statistical information about the test results.

<div class="MoveIt">
  <a>
    <img src="/img/RT8.png" alt="RT8" style={{ width: '600', height: 'auto' }} />
  </a>
</div>

---

## 2. ROS2-Humble

It is easy to be installed by learning ROS2 documentation: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
<div class="MoveIt">
  <a>
    <img src="/img/humble.png" alt="humble" style={{ width: '800', height: 'auto' }} />
  </a>
</div>
---

## 3. ROS2-Control

 It is easy to be installed by learning this [link](https://control.ros.org/rolling/doc/getting_started/getting_started.html#binary-packages) 

---
## 4. MoveIt2

 It is easy to be installed by learning this [link](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)

 ---
## 5. IGH-EtherCAT

 ### 5.1 Installing EtherLab  
 It is easy to be installed by learning this [link](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)

 ### 5.2 Building ethercat_driver_ros2  

 It is easy to be installed by learning this [link](https://icube-robotics.github.io/ethercat_driver_ros2/quickstart/installation.html#building-ethercat-driver-ros2)

 ---

## 6. ROS2_eRob_MoveIt

```bash
mkdir eRob_moveit/src
cd  eRob_moveit/src
git clone https://github.com/ZeroErrControl/eRob_ROS2_MoveIt
cd ROS2_eRob_MoveIt
colcon build 

source ./install/seup.bash

ros2 launch erob_position_control demo.launch.py
```
