# A ros gazebo plugin for pedestrians

This is a ros pkg for gazebo [actor](http://gazebosim.org/tutorials?tut=actor&cat=build_robot) plugin.

### Dependencies
* Ubuntu 16.04
* ROS-kinetic
* Gazebo 8 (with actor suport)
* python-lxml
* turtlebot3
* turtlebot3_msgs
* turtlebot3_simulations

### Build

1. Add the repositories of [Gazebo 8](http://gazebosim.org/tutorials?tut=install_ubuntu) and [ROS kinetic](http://wiki.ros.org/indigo/Installation/Ubuntu)    

2. Install Gazebo 8, Ros kinetic in buntu 16.04 and other dependencies.
```
sudo apt-get install ros-kinetic-desktop-full
sudo apt-get install ros-kinetic-gazebo8-ros-pkgs
```

3. Build packages
```
cd /path/to/workspace/src
git clone git@github.com:onlytailei/gym_ped_sim.git
catkin build
```

![rviz](./rviz_view.png)
![gazebo](./gazebo_view.png)

### Example
```
roslaunch turtlebot3_social default.launch
```

### Node Details
- **actor_plugin**    
  Build based on a [Gazebo official example](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i6). This node broadcasts the tf of every actor. [Social force model](http://vision.cse.psu.edu/courses/Tracking/vlpr12/HelbingSocialForceModel95.pdf) is applied in every actor to interactive with each other.

- **actor_services**    
  The python files help to create several gazebo sdf files quickly. There is a rviz file for visualization.

- **turtlebo3_social**    
  In our socially compliant pedestrian simulator, we collect data by mounting a depth sensor onto one of the pedestrians, to the height matching that of real-world setups. Then, the social force model, as described in the paper, is used to label each incoming depth
image with their corresponding social force.

- **data_collection**    
  To save the related dataset.

### Data Set
The collected [pedestrian navigation dataset](https://drive.google.com/open?id=0Bz6_GtsnLN8zZXFKTHNibHVrMlk) contains:
- depth image
- rgb image
- target 
- social force classification
- social force
- sum force

## Interactive interface
Please reference [gym_style_gazebo](https://github.com/onlytailei/gym_style_gazebo)


------

This is the reference implementation of the plugins and for the paper **Socially-compliant Navigation through Raw Depth Inputs with Generative Adversarial Imitation Learning**. 
If it helps your research, please cite:
```
@article{tai2017socially,
  title={Socially-compliant Navigation through Raw Depth Inputs with Generative Adversarial Imitation Learning},
  author={Tai, Lei and Zhang, Jingwei and Liu, Ming and Burgard, Wolfram},
  journal={arXiv preprint arXiv:1710.02543},
  year={2017}
}
```

