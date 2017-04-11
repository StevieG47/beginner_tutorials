# ROS Beginner Tutorials Week 11 HW 

### Overview

- The talker node was modified in this branch to broadcast a tf frame using the tf library. 

- An integration test was added to test the talker node, specifically the service. 

- The launch file was modified to include an option to record using rosbag. An example is shown where rosbag play is used to replay topic messages, which are picked up by the listener node.


### Dependencies
- ROS Indigo
- catkin
- roscpp package
- std_msgs package
- message_genration package
- tf package
- rostest


### How to Build
 
- Create the catkin workspace and build with:
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone -b Week11_HW --recursive https://github.com/StevieG47/beginner_tutorials
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

To run the launch file without specifying frequency, run the above commands without *talkFreq:=#*


### Tranforms with tf Library

- The talker node was modified to broadcast a tf frame called *talk* with its parent *world*. The transform added has a translation and a rotation. A pdf was created and added to the reposiory showing the the frames. 

- To view transform:
First open a terminal to run the talker node as described previously. After, open a new terminal:
```
rosrun tf tf_echo world talk
```
The transform should be visible on the console, the output will look like the image below:

![tfecho](https://cloud.githubusercontent.com/assets/25371934/24913687/e8ae9d2e-1e9f-11e7-99d8-0d6abe8b4c15.JPG)

The translation and rotation is written to the console. 

- To view a tree of the frames, run the talker node, then in a new terminal:
```
rosrun rqt_tf_tree rqt_tf_tree
```
A visual of the frame tree will come up, and should look like the image below:

![rqttftree](https://cloud.githubusercontent.com/assets/25371934/24913821/5aa1d626-1ea0-11e7-8ec2-8323b9c73298.JPG)

The tree shows the parent frame *world* and child frame *talk*

### Running Test

- An integration test was added to test the talker node, specifically the service described above. The test source file and launch file are in the *test* directory

- To run the test, navigate to your *catkin_ws* then:
```
catkin_make run _tests
```
The output of the tests should look like the image below:

![gtest](https://cloud.githubusercontent.com/assets/25371934/24914068/17b317b6-1ea1-11e7-96bb-5d716374cac0.JPG)

The test creates a client to the service, and then tests whether the client was successfully made. 

### Bag File Recording

- The launch file used to run the talker and listener nodes was modified to also run rosbag to record. When running the launch file, using rosbag to record can be enabled or disabled. 

- By default, the launch file will not record. To do this, first build then:
```
source devel/setup.bash
roslaunch beginner_tutorials begTutorial.launch
```

This will run the talker and listener node like before, with no recording. 

- To enable recording, navigate to catkin_ws:
```
source devel/setup.bash
roslaunch beginner_tutorials begTutorial.launch startRecord:=true
```

The output after running the launch file should look like:

![roslaunch_bagoutput](https://cloud.githubusercontent.com/assets/25371934/24915369/f32de912-1ea4-11e7-95ff-9b901ba5ce30.JPG)


This will record and save the bag file as *File.bag*. The bag file will be saved in the *.ros* folder, not in the current directory. To get information about the bag file:
```
cd ~.ros
rosbag info File.bag
```
This will display the information about the bag file, the output should look something like this:

![rosbaginfo](https://cloud.githubusercontent.com/assets/25371934/24915464/35ed6732-1ea5-11e7-9f6a-479f3b5736e4.JPG)


### Bag File Demonstration

- Using the bag file can be demonstrated by using it with the listener node. First, run the launch file with the *startRecord:=true* argument as seen above. This will record the chatter topic. 

- Next, after stopping the talker and listener nodes from the launch, open a terminal and open only the listener node, as described previously. 

- Navigate to ~.ros directory, where the bag file *File.bag* was saved and run:
```
rosbag play File.bag
```
The recording will play and the listener node should output the message as if talker was running. 
Output of rosbag play:

![rosbag-play](https://cloud.githubusercontent.com/assets/25371934/24921881/54b48cc6-1eba-11e7-8efa-b5bd89d30ee0.JPG)







