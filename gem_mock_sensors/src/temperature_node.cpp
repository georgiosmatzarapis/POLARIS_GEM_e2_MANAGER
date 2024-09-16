// author: georgiosmatzarapis

#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "temperature_node");
  ros::NodeHandle nodeHandle{};

  ros::Publisher temperaturePublisher{
      nodeHandle.advertise<std_msgs::Float32>("/mock/temperature", 1000)};

  ros::Rate loopRate{1};

  ROS_INFO("Temperature node started");

  float temperature{30.0f};
  const ros::Time startTime{ros::Time::now()};

  while (ros::ok()) {
    std_msgs::Float32 temperatureMsg;
    temperatureMsg.data = temperature;

    if (ros::Time::now() - startTime < ros::Duration(30.0)) {
      temperature += 0.83;
    } else if (temperature < 60.0) {
      temperature = 60.0;
    }

    temperaturePublisher.publish(temperatureMsg);

    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}
