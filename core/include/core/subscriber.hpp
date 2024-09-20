// author: georgiosmatzarapis

#pragma once

#include <functional>

namespace core {

template <typename MessageType>
class Subscriber {
 public:
  virtual void
  subscribe(const std::string& topic, unsigned int queueSize,
            std::function<void(const typename MessageType::ConstPtr&)>
                userCallback) = 0;

  virtual ~Subscriber() = default;
};

} // namespace core
