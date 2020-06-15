# ROS2 Installation Mint 19.3

```
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

```
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-latest.list'
```

[Download Dashing Zip](https://github.com/ros2/ros2/releases/download/release-dashing-20200319/ros2-dashing-20200319-linux-bionic-amd64.tar.bz2)

```
mkdir -p ~/ros2_dashing
cd ~/ros2_dashing
tar xf ~/Downloads/ros2-dashing-linux-x86_64.tar.bz2  ``# Your directory
```

```
sudo apt update
sudo apt install -y python-rosdep
sudo apt-get install build-essential
sudo rosdep init
rosdep update
```

```
rosdep install --from-paths ros2-linux/share --ignore-src --rosdistro dashing -y --os=ubuntu:bionic --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"
```

```
sudo apt install -y libpython3-dev
```

## Environment Setup

* source the environment like this, every time you open a new terminal
```
. ~/ros2_dashing/ros2-linux/setup.bash
```

* or add it to bash startup by default
```
echo ". ~/ros2_dashing/ros2-linux/setup.bash" >> ~/.bashrc
```

## Workspace Setup

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

## Create Package

Should be done under ``~/dev_ws/src``

```
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

## Build and Install Package
Should be done under ``~/dev_ws``

```
colcon build
. install/setup.bash
```


## Author

Mridul Pareek

## Version History

* 0.1
    * Initial Commit - Skitbot package migrated to ros2


## Further Reading

* [ROS2 Official tutorials](https://index.ros.org/doc/ros2/Tutorials/)
