# ROS Beginner Tutorials 

### Overview

- These beginner tutorials explore the basics of ROS including packages, nodes, topics, and messages. The tutorials were followed from: http://wiki.ros.org/ROS/Tutorials. 

- This tutorial consists of a publisher node named talker, and a subscriber node named listener. The talker node publishes a message of type std_msgs/String on the chatter topic. The listener node is subscribed to the chatter topic. When a message comes from talker, listener will get the information. The publisher node prints a string, and the listener node prints "I heard" followed by the string. The string in this tutorial is "Steven Gambino, ENPM808X."

### How to Build
 
-Create the catkin workspace and build with:
```
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_make
cd src
git clone --recursive https://github.com/StevieG47/beginner_tutorials
```

### How to Run
- Open a terminal and run: ```roscore```

#### Run publisher node:
- Open another terminal:
```
cd ~catkin_ws/src
source beginner_tutorials/devel/setup.bash
rosrun beginner_tutorials talker
```

- The output when running should look like this:

![talker output](https://cloud.githubusercontent.com/assets/25371934/24433456/cf0f2752-13f6-11e7-89f4-c15ca678c98c.JPG)



#### Run subsriber node:
- Open another terminal:
```
cd ~catkin_ws/src
source beginner_tutorials/devel/setup.bash
rosrun beginner_tutorials listener
```
- The output when running should look like this: 

![listener output](https://cloud.githubusercontent.com/assets/25371934/24433450/c4167bc0-13f6-11e7-884a-628734a7f32d.JPG)


### Dependencies
- ROS Indigo
- catkin
- roscpp package
- std_msgs package
