# ROS_Action_UR5

This repository is an implementation of a ROS2 action for a Universal Robots 5 (UR5) robotic arm (more specifically to be used with [**pla10/ros2_ur5_interface**](https://github.com/pla10/ros2_ur5_interface)). It mainly features an ```UR5.action``` file that defines the structure of three available types of messages: goal, result, and feedback.

## Index

- [Repository Structure](#repository-structure)
- [Instructions for Available Docker Implementation](#instructions-for-using-the-docker-containers)
- [Instructions for configuring the packages manually](#instructions-for-configuring-the-packages-manually)
- [How to initialize client and server comunication](#how-to-initialize-client-and-server-comunication)

---
## Repository Structure

a

---
## Instructions for using the docker containers

The following explanation shows how to initialize the two available containers for the simulation.

---
## Instructions for configuring the packages manually

- Enter ```/ros2ws``` folder and create two packages on the src folder.

- To create the package with the action, use:

```
ros2 pkg create --license Apache-2.0 custom_action_interfaces
```

- To create the package with the server, use:

```
ros2 pkg create --dependencies custom_action_interfaces rclcpp rclcpp_action rclcpp_components --license Apache-2.0 -- custom_action_cpp
```

### custom_action_interfaces

- Create an ```action``` folder and add the ```UR5.action``` file.

- Change the ```CMakeLists.txt``` and ```package.xml``` file to the files on this repository.

- Do ```colcon build```.

To test, source and then ```ros2 interface show custom_action_interfaces/action/UR5```.

### custom_action_cpp

- In the src folder add the ```ur5_action_server.cpp``` file. 

- Change the ```CMakeLists.txt``` and ```package.xml``` file to the files on this repository.

- Do ```colcon build```.

To initialize the server, source and then ```ros2 run custom_action_cpp ur5_action_server```.

---
## How to initialize client and server comunication