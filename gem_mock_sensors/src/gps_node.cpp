// author: georgiosmatzarapis

#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_node");
  ros::NodeHandle nodeHandle{};

  ros::Publisher gpsAccuracyPublisher{nodeHandle.advertise<std_msgs::Float32>(
      "/gem_manager/gps_accuracy", 1000)};

  ros::Rate loopRate{1};

  ROS_INFO("GPS node started");

  float gpsAccuracy{100.0f};
  ros::Time startTime{ros::Time::now()};
  bool lowAccuracyMode{false};

  while (ros::ok()) {
    std_msgs::Float32 gpsAccuracyMsg;

    if (ros::Time::now() - startTime > ros::Duration(20.0)) {
      lowAccuracyMode = !lowAccuracyMode;
      startTime = ros::Time::now();

      if (lowAccuracyMode) {
        ROS_INFO("Switching to low accuracy mode");
      } else {
        ROS_INFO("Switching to high accuracy mode");
      }
    }

    gpsAccuracy = lowAccuracyMode ? 600.0f : 100.0f;

    gpsAccuracyMsg.data = gpsAccuracy;
    gpsAccuracyPublisher.publish(gpsAccuracyMsg);

    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}
