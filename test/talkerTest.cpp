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
 * @file talkerTest.cpp
 * @author Steven Gambino
 * @brief integration test with gtest
 *
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/NewMessage.h"

/**
 * @brief Tests client exists for the NewMessage service
 * @param none
 * @return none
 */
TEST(TestSuite, serviceTest) {
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient < beginner_tutorials::NewMessage
      > (
      "update_service");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}

/**
 * @brief Runs the gtests
 * @param argc argv
 * @return runs tests
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "talkerTest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
