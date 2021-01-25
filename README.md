# self-driving_visualization_ROS

---
## Abastrct
This repo impelemnt a simple self-driving surface for images and point cloud based on ROS 
## Requirement
* ROS melodic version
* Rviz

## Testing on KITTI
Bascially, this code run on KITTI datasets but you also can customize it to fit your own
data
## Result
---
Testing on KITTI raw data

[video](https://youtu.be/WwhYVcieMdo)

## Usage
---

### First. install ROS

Setup your sources.list

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
Setup your keys

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
```
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
```
installation
```
sudo apt update
```
```
sudo apt install ros-melodic-desktop-full
```
setup your bash


```
vim ~/.bashrc
#add this two command at bottom of the bash file
source /opt/ros/melodic/setup.bash
source /home/s5620/catkin_ws/devel/setup.bash
#source your bash to update status
source ~/.bashrc
```
install dependencies
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```
initialize rosdep
```
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```
creat your ROS Workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

### Second. 

download this pakage and compile it
```
cd ~/catkin_make/src
git clone https://github.com/s56207824inc/self-driving_visualization_ROS
cd ..
catkin_make
```
run python script
```
python visualize_for_lidar_img.py 
--img_path /path/to/your images dir 
--pt_path /path/to/your point cloud dir 
--gt_path /path/to/your gt label 
--calib_path /path/to/your calib dir 
```




