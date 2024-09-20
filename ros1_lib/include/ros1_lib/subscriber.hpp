// author: georgiosmatzarapis

#pragma once

#include <ros/ros.h>

#include "core/subscriber.hpp"

namespace ros1_lib {

template <typename MessageType>
class Subscriber : public core::Subscriber<MessageType> {
 public:
  explicit Subscriber(ros::NodeHandle& nodeHandle);

  void subscribe(const std::string& topicName, unsigned int queueSize,
                 std::function<void(const typename MessageType::ConstPtr&)>
                     userCallback) override;

  ~Subscriber() override = default;

 private:
  ros::NodeHandle& nodeHandle_;
  ros::Subscriber subscriber_;
};

template <typename MessageType>
Subscriber<MessageType>::Subscriber(ros::NodeHandle& nodeHandle)
    : nodeHandle_{nodeHandle} {}

template <typename MessageType>
void Subscriber<MessageType>::subscribe(
    const std::string& topicName, unsigned int queueSize,
    std::function<void(const typename MessageType::ConstPtr&)> userCallback) {
  subscriber_ =
      nodeHandle_.subscribe<MessageType>(topicName, queueSize, userCallback);
}

} // namespace ros1_lib
