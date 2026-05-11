// include/waiter_robot/text_utils.hpp
#ifndef WAITER_ROBOT__TEXT_UTILS_HPP_
#define WAITER_ROBOT__TEXT_UTILS_HPP_

#include <string>
#include <behaviortree_cpp/blackboard.h>

namespace waiter_robot
{
/**
 * @brief Sustituye las variables {nombre} de un texto con los valores del blackboard.
 *
 * Ejemplo:
 *   blackboard["bebida"] = "cafe"
 *   formatText("Aqui tiene su {bebida}", bb)  ->  "Aqui tiene su cafe"
 */
std::string formatText(const std::string & text, BT::Blackboard::Ptr blackboard);

}  // namespace waiter_robot

#endif  // WAITER_ROBOT__TEXT_UTILS_HPP_