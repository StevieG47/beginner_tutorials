/* MIT License
 Copyright (c) 2017 Steven Gambino
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 * @copyright Copyright 2017 Steven Gambino
 * @file talker.cpp
 * @author Steven Gambino
 * @brief Talker node to publish to chatter topic with message.
 *
 */

#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <sstream>
#include <cstdlib>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/NewMessage.h"



// Message from master, message without service
std::string message = "Steven Gambino, ENPM808X";  // NOLINT
/**
 * @brief Updates message to subsribe, send to listener
 * @param request and response from srv
 * @return true
 */
bool update(beginner_tutorials::NewMessage::Request &req,  // NOLINT
    beginner_tutorials::NewMessage::Response &res) {  // NOLINT
      message = req.messReq;
      res.messResp = message;
  ROS_INFO_STREAM("Updating with new message");
      return true;
    }

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
/**
 * @brief talker node
 * @param argc argv
 * @return 0
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;


  ros::Publisher chatter_pub = n.advertise < std_msgs::String  // publisher
      > ("chatter", 1000);

  int rate = 10;  // defualt of 10
  if (argc == 2) {  // if argument is given from launch arc will equal 2, otherwise 1
    if (atoi(argv[1]) < 0) {
      ROS_ERROR_STREAM("Negative Frequency Entered");
    }
    rate = atoi(argv[1]);  // argv[1] is the argument talkFreq when we say talkFreq:=value in roslaunch command
    ROS_DEBUG_STREAM("Frequency changed to " << rate);
  }

  if (rate < 3) {
    ROS_WARN_STREAM("publisher frequency is slow");
  }

  ros::Rate loop_rate(rate);  // set the rate
  ROS_INFO_STREAM("Currrent Rate: " << rate);

  ros::ServiceServer service = n.advertiseService("update_service", update);  // Server

  // TF broadcast
  static tf::TransformBroadcaster br;
  tf::Transform transform;


  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << message << " " << count;  // Whatever the message is, will change depending on service
    msg.data = ss.str();

    ROS_INFO_STREAM(msg.data.c_str());
    ROS_DEBUG_STREAM("Current Rate: " << rate);


    // ROS_INFO_STREAM("Currrent Rate: " << rate);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    // TF broadcaster
    transform.setOrigin(tf::Vector3(1.0, 2.0, 3.0));  // set translation
    tf::Quaternion q;
    q.setRPY(10, 20, 30);  // set rotations
    transform.setRotation(q);
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));  // broadcast talk with parent world

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
