// author: georgiosmatzarapis

#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "signal_node");
  ros::NodeHandle nodeHandle{};

  ros::Publisher signalPublisher{
      nodeHandle.advertise<std_msgs::Int32>("/mock/signal_strength", 1000)};

  ros::Rate loopRate{1};

  ROS_INFO("Signal node started");

  constexpr float stableDuration{5.0f};
  constexpr float lowSignalDuration{35.0f};
  constexpr float lostSignalDuration{5.0f};

  float signalStrength{1.0f};
  ros::Time startTime{ros::Time::now()};
  enum SignalState { CONNECTED, LOW_SIGNAL, LOST_SIGNAL };
  SignalState currentState{CONNECTED};

  while (ros::ok()) {
    std_msgs::Int32 signalMsg;

    switch (currentState) {
      case CONNECTED:
        if (ros::Time::now() - startTime >= ros::Duration(stableDuration)) {
          signalStrength = 2.0f;
          currentState = LOW_SIGNAL;
          startTime = ros::Time::now();
          ROS_INFO("Switching to low signal");
        }
        break;
      case LOW_SIGNAL:
        if (ros::Time::now() - startTime >= ros::Duration(lowSignalDuration)) {
          signalStrength = 0.0f;
          currentState = LOST_SIGNAL;
          startTime = ros::Time::now();
          ROS_INFO("Switching to lost signal");
        }
        break;
      case LOST_SIGNAL:
        if (ros::Time::now() - startTime >= ros::Duration(lostSignalDuration)) {
          signalStrength = 1.0f;
          currentState = CONNECTED;
          startTime = ros::Time::now();
          ROS_INFO("Switching to stable signal");
        }
        break;
    }

    signalMsg.data = signalStrength;
    signalPublisher.publish(signalMsg);

    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}
