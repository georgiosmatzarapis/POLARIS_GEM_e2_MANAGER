// author: georgiosmatzarapis

#pragma once

namespace core {

template <typename MessageType>
class Publisher {
 public:
  virtual void publish(const MessageType& message) = 0;

  virtual ~Publisher() = default;
};

} // namespace core
