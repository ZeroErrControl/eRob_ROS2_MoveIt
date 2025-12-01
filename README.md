***This is an open-source demo related to the eRob product, provided solely for reference by developers. Please note that issues within the open-source project are independent of the quality of eRob products. Users are advised to exercise caution while using the demo. We are not responsible for any damage caused by improper operations. For any project errors, please raise a query in the Issues section. Collaboration and forks to resolve open-source project issues are welcome.***

# eRob ROS2 MoveIt
---
The new versions of erob_position_control and erobo3_control have added timeout settings and provide a solution to address the issue where eRob cannot enter DC mode in the ethercat_ros2 interface.
[result](https://cdn-fusion.imgcdn.store/i/2024/e4c73fbe2dcfa4fc.png)
<div class="result">
  <a>
    <img src="https://cdn-fusion.imgcdn.store/i/2024/e4c73fbe2dcfa4fc.png" alt="result" style={{ width: '1000', height: 'auto' }} />
  </a>
</div>


# Installation Required Dependencies

---
## 1. Installing the Preempt-RT Real-Time Patch on Ubuntu 22.04

### 1.1 Check Kernel Version and Install Necessary Packages

```bash
uname -a
```
<div class="MoveIt"> <a> <img src="https://cdn-fusion.imgcdn.store/i/2024/ac83b01153d11553.png" alt="RT1" style="width:900px; height: auto;" /> </a> </div> 


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
    <img src="https://cdn-fusion.imgcdn.store/i/2024/f806094e23085a11.png" alt="RT2" style={{ width: '1200', height: 'auto' }} />
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
vim .config
```

``` txt
CONFIG_SYSTEM_TRUSTED_KEYS=""

CONFIG_SYSTEM_REVOCATION_KEYS=""
```
<div class="MoveIt">
  <a>
    <img src="https://cdn-fusion.imgcdn.store/i/2024/0e908b9fac95de45.png" alt="RT3" style={{ width: '600', height: 'auto' }} />
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
    <img src="https://cdn-fusion.imgcdn.store/i/2024/6d937b8f639f12d3.png" alt="RT5" style={{ width: '600', height: 'auto' }} />
  </a>
</div>

<div class="MoveIt">
  <a>
    <img src="https://cdn-fusion.imgcdn.store/i/2024/ceb860804d6fcc45.png" alt="RT6" style={{ width: '600', height: 'auto' }} />
  </a>
</div>

<div class="MoveIt">
  <a>
    <img src="https://cdn-fusion.imgcdn.store/i/2024/50af0abbc109a2fa.png" alt="RT7" style={{ width: '600', height: 'auto' }} />
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
    <img src="https://cdn-fusion.imgcdn.store/i/2024/602b07696a47f642.png" alt="RT8" style={{ width: '600', height: 'auto' }} />
  </a>
</div>

---

## 2. ROS2-Humble

It is easy to be installed by learning ROS2 documentation: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
<div class="MoveIt">
  <a>
    <img src="https://cdn-fusion.imgcdn.store/i/2024/4c049cd7485dce21.png" alt="humble" style={{ width: '800', height: 'auto' }} />
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
cd ..
colcon build 

source ./install/setup.bash

ros2 launch erob_position_control demo.launch.py
ros2 launch erobo3_control demo.launch.py
ros2 launch erobo5 demo.launch.py
ros2 launch erobo10 demo.launch.py


```

---

## 7. Connecting to Physical Robots (erobo5 & erobo10)

Refer to erobo3_control for creation and modification.

---


