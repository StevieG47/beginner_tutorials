# ROS Beginner Tutorials Week 10 HW Branch

### Overview

- The talker/listener nodes were modified in this branch to include a service that, when called, changes the output message.

- A launch file was also added to run the talker and listener at the same time. When running the launch file from the command line an argument can also be given to change the publish frequency.

- All five logging levels are now included between the two nodes. Some screenshots of rqt_console are also included.


### Dependencies
- ROS Indigo
- catkin
- roscpp package
- std_msgs package


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
source devel/setup.bash
rosrun beginner_tutorials talker
```

- The output when running should look like this:

![talker output](https://cloud.githubusercontent.com/assets/25371934/24433456/cf0f2752-13f6-11e7-89f4-c15ca678c98c.JPG)



#### Run subscriber node:
- Open another terminal:
```
cd ~catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```
- The output when running should look like this: 

![listener output](https://cloud.githubusercontent.com/assets/25371934/24433450/c4167bc0-13f6-11e7-884a-628734a7f32d.JPG)


#### Calling Service
- A service has been added in the talker node to change the message.

- To call service, first follow the steps above to have the talker/listener nodes up and running. Then open a new terminal.

- View services with:
```rosservice list```
The service */update_service* should be one of the listed.

- Call service with:
```
cd ~catkin_ws
source devel/setup.bash
rosservice call /update_service NewString
``` 
where NewString is the new message to write. No quotes are needed. 

-An example output is seen below.

#### Using Launch File
- Using the launch file *begTutorial.launch*, The nodes can be started at once using one command. Also, running the launch file and giving an argument *talkFreq*, the publish frequency can be changed.

- To use launch file:
```
cd ~catkin_ws
roslaunch beginner_tutorials begTutorial.launch talkFreq:=4
```
talkFreq:=# sets the publish frequency, so if using the line above, the publish frequency would be equal to 4.

- After running, a new window with the listener node output will pop up, and the original terminal will have the talker node. The talker node output should like something like this:


