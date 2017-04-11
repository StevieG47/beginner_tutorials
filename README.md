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


### Calling Service
- A service has been added in the talker node to change the message.

- To call service, first follow the steps above to have the talker/listener nodes up and running. Then open a new terminal.

- View services with:
*rosservice list*
The service */update_service* should be one of the listed.

- Call service with:
```
cd ~catkin_ws
source devel/setup.bash
rosservice call /update_service NewString
``` 
where NewString is the new message to write. No quotes are needed. 

-An example output is seen below.

![servicescapture](https://cloud.githubusercontent.com/assets/25371934/24688782/4880f5fe-1990-11e7-9e60-b9abfd9c79fd.JPG)


### Using Launch File
- Using the launch file *begTutorial.launch*, The nodes can be started at once using one command. Also, running the launch file and giving an argument *talkFreq*, the publish frequency can be changed.

- To use launch file make sure roscore is running, then:
```
cd ~catkin_ws
roslaunch beginner_tutorials begTutorial.launch talkFreq:=4
```
talkFreq:=# sets the publish frequency, so if using the line above, the publish frequency would be equal to 4.

- After running, a new window with the listener node output will pop up, and the original terminal will have the talker node. The talker node output should like something like this:

![launchfiletalk](https://cloud.githubusercontent.com/assets/25371934/24688793/5d49092c-1990-11e7-94ce-75f82fa48d86.JPG)


### TF
there are two frames, world and talk, talk was included in talker.cpp, in talker node. To see the translation, rotation rosrun beginner_tutorials talker then in new window rosrun tf tf_echo world talk and then info should be constantly displayed showing translation and rotation in quaternion and whatever. 

### Test
Tests directory added with a test node. The test is the service NewMessage that was previously made. A client is made to that service and the clients existence is tested. Passing means that service has a client, nothing went wrong. Run with catkin_make run_tests, it should build then do the test and let you know if it was a success or failure.
### Bag
Default on the launch file is false so run launch command with no arguments won't have it record. Running it then putting startRecord:=true after will enable the rosbag record -a and record everything. The bag file is saved in the .ros folder (in home) and is called File.bag




