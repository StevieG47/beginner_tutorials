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

