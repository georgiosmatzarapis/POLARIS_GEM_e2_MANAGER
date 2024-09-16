// author: georgiosmatzarapis

#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "battery_node");
  ros::NodeHandle nodeHandle{};

  ros::Publisher batteryPublisher{
      nodeHandle.advertise<std_msgs::Float32>("/mock/battery_level", 1000)};

  ros::Rate loopRate{1};

  ROS_INFO("Battery node started");

  float batteryLevel{100.0f};
  const ros::Time startTime{ros::Time::now()};

  while (ros::ok()) {
    std_msgs::Float32 batteryMsg;
    batteryMsg.data = batteryLevel;

    if (ros::Time::now() - startTime < ros::Duration(30.0)) {
      batteryLevel -= 1.6;
    } else if (batteryLevel > 50.0) {
      batteryLevel = 49.0;
    }

    batteryPublisher.publish(batteryMsg);

    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}
