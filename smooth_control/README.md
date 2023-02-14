# Smooth Control
Smooth Planner

---

### Test
open a new terminal.
````
$ roscore
$ cd catkin_ws/
$ catkin_make
$ source devel/setup.sh
$ rosrun smooth_control smooth_control
````

---
### Update Logs
>TODO

---
### Parameter
| Name                 | Type   | Access | Function                   |
| :--------------------|:------:| :-----:|:---------------------------|

---
### Topic
| Name                  | Type                       | SUB/PUB | Function                   |
| :---------------------|:--------------------------:| :------:|:---------------------------|
| `/untreated_cmd_vel`        | geometry_msgs::Twist   | SUB     | Untreated Velocity                |
| `/cmd_vel`        | geometry_msgs::Twist          | PUB     | Treated Velocity          |
---
### Topic Info
| Name                 | Enum        |  Function                       |
| :--------------------|:-----------:| :-------------------------------|

---

### Dependency 
##### ROS Libraries : ROS Melodic CPP
##### Other Libraries : NONE

---


