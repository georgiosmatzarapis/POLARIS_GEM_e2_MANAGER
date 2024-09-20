// author: georgiosmatzarapis

#pragma once

#include <ros/ros.h>

#include "core/publisher.hpp"

namespace ros1_lib {

template <typename MessageType>
class Publisher : public core::Publisher<MessageType> {
 public:
  explicit Publisher(const std::string& topicName, unsigned int queueSize);
  explicit Publisher(ros::NodeHandle& nodeHandle, const std::string& topicName,
                     unsigned int queueSize);

  void publish(const MessageType& message) override;

  ~Publisher() override = default;

 private:
  ros::NodeHandle nodeHandle_{};
  ros::Publisher publisher_;
};

template <typename MessageType>
Publisher<MessageType>::Publisher(const std::string& topicName,
                                  unsigned int queueSize)
    : publisher_{nodeHandle_.advertise<MessageType>(topicName, queueSize)} {}

template <typename MessageType>
Publisher<MessageType>::Publisher(ros::NodeHandle& nodeHandle,
                                  const std::string& topicName,
                                  unsigned int queueSize)
    : publisher_{nodeHandle.advertise<MessageType>(topicName, queueSize)} {}

template <typename MessageType>
void Publisher<MessageType>::publish(const MessageType& message) {
  publisher_.publish(message);
}

} // namespace ros1_lib
