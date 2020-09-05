## PR2 Robot 3D Perception
This project gives the PR2 robot the ability to locate an object in a cluttered environment, pick it up and then move it to some other location. This is an interesting problem to solve and is a challenge at the forefront of the robotics industry today.
The project uses a perception pipeline to identify target objects from a so-called “Pick-List” in that particular order, pick up those objects and place them in corresponding drop-boxes.

Note : The perception pipeline for this project is explained in PART1 and PART2 of another repo [here](https://github.com/ashutoshtiwari13/ROS-PCL-Segmentation)

![SIM RUN]()
### Pick and Place task Simulation
For this project, we have a variety of different objects to identify. There are 3 different worlds
- test1.world
- test2.world
- test3.world
Each of the three worlds has different items on the table in front of the robot. These worlds are located in the `/pr2_robot/ worlds/`. Object recognition can be performed base on the train model.sav generated from PART2 of the mini-projetc described  [here](https://github.com/ashutoshtiwari13/ROS-PCL-Segmentation). The program read each pick list which are `pick_list1.yaml` `pick_list_2.yaml` and `pick_list_3.yaml` and display the name of each objects.
