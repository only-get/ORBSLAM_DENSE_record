# 一、安装依赖

## 1.安装ROS依赖包

```shell
sudo apt-get install -y ros-melodic-navigation
sudo apt-get install -y ros-melodic-robot-localization
sudo apt-get install -y ros-melodic-robot-state-publisher
```

安装处出现【无法定位软件包】，是因为安装的依赖跟ubuntu版本不对应。比如ubuntu18.04对应melodic，将对应名称修改即可解决。

如果没安装或者没有正确安装依赖，可能出现下述问题

```shell
process[robot_state_publisher-5]: started with pid [4647]
ERROR: cannot launch node of type [robot_localization/ekf_localization_node]: robot_localization
ROS path [0]=/opt/ros/melodic/share/ros
ROS path [1]=/home/yinton/catkin_ws/src
ROS path [2]=/opt/ros/melodic/share
ERROR: cannot launch node of type [robot_localization/navsat_transform_node]: robot_localization
ROS path [0]=/opt/ros/melodic/share/ros
ROS path [1]=/home/yinton/catkin_ws/src
ROS path [2]=/opt/ros/melodic/share
process[lio_sam_rviz-8]: started with pid [4648]
```

## 2.安装GTSAM 4.0.2 （因子图库）


```bash
下载地址 https://github.com/borglab/gtsam 找到4.0.2版本，下载，解压
cd gtsam
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
sudo make install -j8
```

据说非4.0.2版本容易出问题。若更换gtsam版本，重新编译即可覆盖。记得在catkin_make之前删除重复的gtsam包，不然无法编译。

<font color ='red'>选择eigen版本，虽然在此处不会出错，但是会在编译lio-sam时会出现版本冲突的错误</font>

```cmake
###############################################################################
# Option for using system Eigen or GTSAM-bundled Eigen
### These patches only affect usage of MKL. If you want to enable MKL, you *must*
### use our patched version of Eigen
### See:  http://eigen.tuxfamily.org/bz/show_bug.cgi?id=704 (Householder QR MKL selection)
###       http://eigen.tuxfamily.org/bz/show_bug.cgi?id=705 (Fix MKL LLT return code)
option(GTSAM_USE_SYSTEM_EIGEN "Find and use system-installed Eigen. If 'off', use the one bundled with GTSAM" OFF)
option(GTSAM_WITH_EIGEN_UNSUPPORTED "Install Eigen's unsupported modules" OFF)
set(GTSAM_USE_SYSTEM_EIGEN ON)                ##需要添加使用系统安装的eigen
# Switch for using system Eigen or GTSAM-bundled Eigen
if(GTSAM_USE_SYSTEM_EIGEN)
	find_package(Eigen3 REQUIRED)

	# Use generic Eigen include paths e.g. <Eigen/Core>
	set(GTSAM_EIGEN_INCLUDE_FOR_INSTALL "${EIGEN3_INCLUDE_DIR}")
	
	# check if MKL is also enabled - can have one or the other, but not both!
	# Note: Eigen >= v3.2.5 includes our patches
	if(EIGEN_USE_MKL_ALL AND (EIGEN3_VERSION VERSION_LESS 3.2.5))
	  message(FATAL_ERROR "MKL requires at least Eigen 3.2.5, and your system appears to have an older version. Disable GTSAM_USE_SYSTEM_EIGEN to use GTSAM's copy of Eigen, or disable GTSAM_WITH_EIGEN_MKL")
	endif()
```

## <font color='green'>可能需要安装</font>

### 安装 TBB

```
 sudo apt-get install libtbb-dev
```

###  安装MKL

[官网链接](https://www.intel.com/content/www/us/en/developer/articles/guide/installing-free-libraries-and-python-apt-repo.html)

sample:

```shell
sudo bash
#<type your user password when prompted.  this will put you in a root shell>
# cd to /tmp where this shell has write permission
cd /tmp
# now get the key:
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
# now install that key
apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
# now remove the public key file exit the root shell
rm GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
exit
```

Add the APT Repository

```shell
sudo sh -c 'echo deb https://apt.repos.intel.com/mkl all main > /etc/apt/sources.list.d/intel-mkl.list'
sudo apt-get update
```

要安装英特尔® 性能库之一的特定版本

```shell
sudo apt-get install intel-mkl-2018.2-046
```

要安装适用于 Python* 的英特尔® 分发版的特定语言版本：

```shell
sudo apt-get install intelpython3
```

# 运行LIO-SAM

下载和编译LIO-SAM

```shell
cd ~/catkin_ws/src
git clone https://github.com/TixiaoShan/LIO-SAM
cd ..
catkin_make
```

参数配置，本文以park.bag为例，该数据集带GPS数据，需要修改如下参数

```text
imuTopic: "imu_raw"
gpsTopic: "odometry/gps"
useImuHeadingInitialization: true
```

运行LIO-SAM

```shell
source devel/setup.bash
roslaunch lio_sam run.launch
rosbag play ~/date/park.bag
```

## <font color='red'>可能出现的错误</font>

运行时报错
```error while loading shared libraries: [libmetis-gtsam.so](https://link.zhihu.com/?target=http%3A//libmetis-gtsam.so/): cannot open shared object file: No such file or directory```

```shell
解决: sudo ln -s /usr/local/lib/libmetis-gtsam.so /usr/lib/libmetis-gtsam.so
```

<font color='red'>**巨坑**</font>
运行时报错 

```bash
[lio_sam_mapOptmization-5] process has died [pid 260348, exit code -11

[lio_sam_mapOptmization-2]

[lio_sam_mapOptmization-3]

```

 <font color='green'>解决:</font>

方法一 、gesam编译时使用，```cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..```

方法二、解决方法

实际问题是库文件libmetis.so 的位置。它是一个运行时库，但是当应用程序查找它时，它的位置不在预期的目录中。在通过运行命令sudo make install -j8安装库gtsam 时，文件libmetis.so安装在/usr/local/lib/的默认位置，但是当我们启动 ros 工作区时，运行时库查看位置/opt/ros/melodic/库/

```shell
cd /usr/local/lib/
sudo cp libmetis.so /opt/ros/melodic/lib/
```

方法三、（实际解决问题）

删除了自己安装的boost1.68

```shell
sudo rm -r /usr/local/include/boost
sudo rm -r /usr/local/lib/libboost*
```

原理不明，因为看得到了编译lio-sam有一个warning可能会发生boost的版本冲突，就试了试，虽然再次编译的时候会出现error```make[2]: *** 没有规则可制作目标“/usr/local/lib/libboost_serialization.so”```,但是只需要将```libboost_serialization```复制到指定路径下即可，
```shell 
sudo cp  /usr/lib/x86_64-linux-gnu/libboost_serialization.so /usr/local/lib/```
```

![2023-05-11 09-40-33屏幕截图](/home/lee/Desktop/LIO-sam_log/picture/2023-05-11 09-40-33屏幕截图.png)