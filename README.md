# ROS Beginner Tutorials Week 10 HW Branch

### Overview

- The talker/listener nodes were modified in this branch to include a service that, when called, changes the output message.

- A launch file was also added to run the talker and listener at the same time. When running the launch file from the command line an argument can also be given to change the publish frequency.

- All five logging levels are now included between the two nodes. Some screenshots of rqt_console are also included.


### How to Build
 
- Create the catkin workspace and build with:
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone -b Week10_HW --recursive https://github.com/StevieG47/beginner_tutorials
cd ..
catkin_make
```

### Running Talker/Listener Nodes
- Open a terminal and run: ```roscore```

#### Run publisher node:
- Open another terminal:
```
cd ~catkin_ws
source beginner_tutorials/devel/setup.bash
rosrun beginner_tutorials talker
```

- The output when running should look like this:

![talker output](https://cloud.githubusercontent.com/assets/25371934/24433456/cf0f2752-13f6-11e7-89f4-c15ca678c98c.JPG)



#### Run subscriber node:
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


## Overview

To build/run:

Assuming there is existing catkin_ws and catkin_ws/src:
Put all these files into src
Go back to catkin_ws
catkin_make
Open 4 terminals
T1: roscore
T2: source devel/setup.bash
    rosrun beginner_tutorials talker
T3: source devel/setup.bash
    rosrun beginner_tutorials listener
T4: source devel/setup.bash
    rosservice list --> to view service, update_service should be there
    
    rosservice call /update_service NewString

Should see talker and listener working like they did before.
Then when service is called the talker/listener message should change to 
whatever your NewString is.


Added launch, to build/run:

Go to catkin_ws (assuming you have catkin_ws/src)

paste files from repo in catkin_ws/src

catkin_make

source devel/setup.bash

roslaunch beginner_tutorials begTutorial.launch talkFreq:=3

ANother window for listener node should open, frequ should be 3


Logger level screenshots added with Debug and Info severity

