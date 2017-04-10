#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>

TEST(TestSuite, serviceTest)
{
  ros::NodeHandle n;
  
  ros::ServiceClient client = n.serviceClient<beginner_tuorials::NewMessage>(
      "update_service);
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,'testnode');
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}