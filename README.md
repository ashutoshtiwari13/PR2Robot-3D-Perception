## PR2 Robot 3D Perception
This project gives the PR2 robot the ability to locate an object in a cluttered environment, pick it up and then move it to some other location. This is an interesting problem to solve and is a challenge at the forefront of the robotics industry today.
The project uses a perception pipeline to identify target objects from a so-called “Pick-List” in that particular order, pick up those objects and place them in corresponding drop-boxes.

Note : The perception pipeline for this project is explained in PART1 and PART2 of another repo [here](https://github.com/ashutoshtiwari13/ROS-PCL-Segmentation).

<p align= "center">
<img src="https://github.com/ashutoshtiwari13/PR2Robot-3D-Perception/blob/master/pr2_robot/run/PR2.gif height="600px" width="400px"/>
</p>

### [Setup](#Setup)
### [Project Heads-Up](#Project-Heads-Up)
### [Pick and Place Simulate](#Pick-and-Place-Simulate)

## Setup
1. Move to the `/src` directory of your active ROS workspace and clone the project files.
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/ashutoshtiwari13/PR2Robot-3D-Perception.git
```
2. Install missing dependencies using rosdep install.
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

3. Build the project.
```sh
$ cd ~/catkin_ws
$ catkin_make
```

4. Add following to your `.bashrc` file
```sh
$ export GAZEBO_MODEL_PATH=~/catkin_ws/src/PR2Robot-3D-Perception/pr2_robot/models
$ source ~/catkin_ws/devel/setup.bash
```
Note : source the `~/.bashrc` file too to avoid errors.

5. See the project scenario using
```sh
$ roslaunch pr2_robot pick_place_project.launch
```

6. To run the demo
```sh
$ cd ~/catkin_ws/src/PR2Robot-3D-Perception/pr2_robot/scripts
$ chmod u+x pr2_safe_spawner.sh
$ ./pr2_safe_spawner.sh
```

## Project Heads-Up
*To be added*
<p align= "center">
<img src="https://github.com/ashutoshtiwari13/PR2Robot-3D-Perception/blob/master/pr2_robot/run/pr2-demo.png height="600px" width="400px"/>
</p>

## Pick and Place Simulate
For this project, we have a variety of different objects to identify. There are 3 different worlds
- test1.world
- test2.world
- test3.world

Each of the three worlds has different items on the table in front of the robot. These worlds are located in the `/pr2_robot/ worlds/`. Object recognition can be performed base on the train model.sav generated from PART2 of the mini-projetc described  [here](https://github.com/ashutoshtiwari13/ROS-PCL-Segmentation). The program read each pick list which are `pick_list1.yaml` `pick_list_2.yaml` and `pick_list_3.yaml` and display the name of each objects.

### Running the 3 world tests

The required test world is selected by editing the following lines in `pick_place_project.launch`

```sh
<arg name="world_name" value="$(find pr2_robot)/worlds/test1.world"/>
```
and
```sh
<rosparam command="load" file="$(find pr2_robot)/config/pick_list_1.yaml"/>
```

### First World Output

<p align= "left">
<img src="https://github.com/ashutoshtiwari13/PR2Robot-3D-Perception/blob/master/pr2_robot/run/world1.jpg height="600px" width="400px"/>
</p>

<p align= "right">
<img src="https://github.com/ashutoshtiwari13/PR2Robot-3D-Perception/blob/master/pr2_robot/run/obj1.png height="600px" width="400px">
</p>
### Second World Output
<p align= "left">
<img src="https://github.com/ashutoshtiwari13/PR2Robot-3D-Perception/blob/master/pr2_robot/run/world2.jpg height="600px" width="400px">
</p>

<p align= "right">
<img src="https://github.com/ashutoshtiwari13/PR2Robot-3D-Perception/blob/master/pr2_robot/run/obj2.png height="600px" width="400px">
</p>

### Third World Output
<p align= "left">
<img src="https://github.com/ashutoshtiwari13/PR2Robot-3D-Perception/blob/master/pr2_robot/run/world3.jpg height="600px" width="400px">
</p>

<p align= "right">
<img src="https://github.com/ashutoshtiwari13/PR2Robot-3D-Perception/blob/master/pr2_robot/run/obj3.png height="600px" width="400px">
</p>
