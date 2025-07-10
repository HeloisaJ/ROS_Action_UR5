# ROS_Action_UR5

## Instructions

- Enter ros2ws folder and create two packages on the src folder.

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