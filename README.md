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
