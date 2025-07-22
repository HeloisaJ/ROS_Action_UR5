# ROS_Action_UR5

This repository is an implementation of a ROS2 action for a Universal Robots 5 (UR5) robotic arm (more specifically to be used with [**pla10/ros2_ur5_interface**](https://github.com/pla10/ros2_ur5_interface)).

## Index

- [Repository Structure](#repository-structure)
- [Repository Features](#features)
- [Instructions for Available Docker Implementation](#instructions-for-available-docker-implementation)
- [Instructions for Configuring the Packages Manually](#instructions-for-configuring-the-packages-manually)
- [How to initialize client and server comunication](#how-to-initialize-client-and-server-comunication)

---
## Repository Structure

```plaintext
.
├── custom_action_cpp/                  # Package containing the action server
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── src/
│       ├── ur5_action_server.cpp       # Server code in C++
├── custom_action_interface/            # Package containing the action definition
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── action/
│       ├── UR5.action                  # Definition of the actions goal, server and feedback
├── docker/                             # Folder containing the image definitions for server and client containers
│   ├── client/
│       ├── Dockerfile                  # Client Dockerfile
│   ├── server/
│       ├── Dockerfile                  # Server Dockerfile
├── scripts/
│   ├── init_client.sh                  # Script for initializing the docker client container
│   ├── init_server.sh                  # Script for initializing the docker server container
├── .gitignore
├── command_start                       # Presents two commands for moving the arm one for topic and one for action
├── README.md                           # Project documentation 
```

---
## Features

This repository mainly features an ```UR5.action``` file that defines the structure of three available types of messages: 

- Goal
- Result
- Feedback

Also, this project presents a server file written in C++ (```ur5_action_server.cpp```) that defines the client commands for UR5 arm movement, and the configuration files (CMake and XML) for the respective packages.

**Disclaimer**: This repository does not include the code for the robotic arm, the code used for the UR5 is available at: [**pla10/ros2_ur5_interface**](https://github.com/pla10/ros2_ur5_interface).

### Goal Structure

- **execute:** A string containing the name of the command for moving the UR5. There are four commands: 

### Result Structure

### Feedback Structure

---
## Instructions for Available Docker Implementation

The following explanation shows how to initialize the two available containers for the simulation.

---
## Instructions for Configuring the Packages Manually

- Enter ```/ros2ws``` folder and create two packages on the src folder.

- To create the package with the action, use:

```
ros2 pkg create --license Apache-2.0 custom_action_interfaces
```

- To create the package with the server, use:

```
ros2 pkg create --dependencies custom_action_interfaces rclcpp rclcpp_action rclcpp_components --license Apache-2.0 -- custom_action_cpp
```

### In the custom_action_interfaces package:

- Create an ```action``` folder and add the ```UR5.action``` file.

- Change the ```CMakeLists.txt``` and ```package.xml``` file to the files on this repository.

- Do ```colcon build```.

To test, source and then ```ros2 interface show custom_action_interfaces/action/UR5```.

### In the custom_action_cpp package:

- In the src folder add the ```ur5_action_server.cpp``` file. 

- Change the ```CMakeLists.txt``` and ```package.xml``` file to the files on this repository.

- Do ```colcon build```.

---
## How to Initialize Client and Server Communication

**Remember to build the code and then source in each terminal.**

If you decided not to use the docker implementation, for communication between client and server on the same subnet, set the ```ROS_DOMAIN_ID``` to a specific number and export the ```ROS_DOMAIN_ID``` for **all terminals**. Example:

```
ROS_DOMAIN_ID=42
echo $ROS_DOMAIN_ID     # See if the ROS_DOMAIN_ID was configured correctly
export ROS_DOMAIN_ID
```

Initialize one of the hosts (docker container or other device), open one terminal and activate the RViz simulation and in another terminal use this command to activate the server:

```
ros2 run custom_action_cpp ur5_action_server
```

In another host, use the structure of the second command in ```command_start```, this will send a message as a client to the server. The available commands are described in the [features topic](#features).

```
ros2 action send_goal /ur5_move custom_action_interfaces/action/UR5 "execute: default"
```

Then observate the move on RViz.