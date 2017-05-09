# A ros gazebo plugin for pedestrians

### Gazebo and Rviz view
<img src="./gazebo_view.png" alt="Drawing" style="width: 100px;"/>
<img src="./rviz_view.png" alt="Drawing" style="width: 105px;"/>

<br>
<br/>

This is a ros pkg for gazebo [actor](http://gazebosim.org/tutorials?tut=actor&cat=build_robot) plugin.

### Dependencies
* Ubuntu 16.04
* Ros kinetic
* Gazebo 8
* python-lxml

### Build

1. Add the repositories of [Gazebo 8](http://gazebosim.org/tutorials?tut=install_ubuntu) and [ROS kinetic](http://wiki.ros.org/indigo/Installation/Ubuntu)    

2. Install Gazebo 8 and Ros kinetic in buntu 16.04.
```
sudo apt-get install ros-kinetic-desktop-full
sudo apt-get install ros-kinetic-gazebo8-ros-pkgs
```

3. Build packages
```
cd /path/to/workspace/src
git clone https://github.com/onlytailei/Gazebo_ROS_Pedestrians.git
catkin build
```

### Example
```
roslaunch actor_services ped_launch.launch
```

### Node Details
- **actor_plugin**    
  Build base on a [Gazebo official example](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i6). You can set/get the position and the target position of the actors through ros services */actorname/SetActorPose* and */actorname/SetActorTarget*. This node also broadcasts the tf of every actor.

- **actor_services**    
  The *random_create.py* helps to create a gazebo sdf file quickly. There are also the launch file and rviz file for test and visualization.
