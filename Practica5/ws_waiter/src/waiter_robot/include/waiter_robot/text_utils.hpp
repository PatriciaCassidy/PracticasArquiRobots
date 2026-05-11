#ifndef WAITER_ROBOT__TEXT_UTILS_HPP_
#define WAITER_ROBOT__TEXT_UTILS_HPP_

#include <string>
#include <behaviortree_cpp/blackboard.h>

namespace waiter_robot {
  std::string formatText(const std::string& text, BT::Blackboard::Ptr blackboard);
}

#endif  // WAITER_ROBOT__TEXT_UTILS_HPP_