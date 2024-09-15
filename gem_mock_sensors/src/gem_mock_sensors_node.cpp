// author: georgiosmatzarapis

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gem_mock_sensors");

  ros::NodeHandle nodeHandle{};

  ros::Publisher batteryPublisher{
      nodeHandle.advertise<std_msgs::Float32>("/mock/battery_level", 1000)};
  ros::Publisher temperaturePublisher{
      nodeHandle.advertise<std_msgs::Float32>("/mock/temperature", 1000)};
  ros::Publisher gpsAccuracyPublisher{
      nodeHandle.advertise<std_msgs::Float32>("/mock/gps_accuracy", 1000)};
  ros::Publisher signalPublisher{
      nodeHandle.advertise<std_msgs::Int32>("/mock/signal_strength", 1000)};
  ros::Publisher emergencyButtonPublisher{
      nodeHandle.advertise<std_msgs::Bool>("/mock/emergency_button", 1000)};

  ros::Rate loopRate{1};

  ROS_INFO("Node started");

  while (ros::ok()) {
    std_msgs::Float32 batteryMsg;
    std_msgs::Float32 temperatureMsg;
    std_msgs::Float32 gpsAccuracyMsg;
    std_msgs::Int32 signalMsg;
    std_msgs::Bool emergencyButtonMsg;

    batteryMsg.data = static_cast<float>(rand() % 101);
    temperatureMsg.data = static_cast<float>(rand() % 81);
    gpsAccuracyMsg.data = static_cast<float>(rand() % 1001);
    signalMsg.data = rand() % 3;
    emergencyButtonMsg.data = (rand() % 100) < 5;

    batteryPublisher.publish(batteryMsg);
    temperaturePublisher.publish(temperatureMsg);
    gpsAccuracyPublisher.publish(gpsAccuracyMsg);
    signalPublisher.publish(signalMsg);
    emergencyButtonPublisher.publish(emergencyButtonMsg);

    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}
