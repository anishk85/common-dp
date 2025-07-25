// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from thor_server:action/JointTask.idl
// generated code does not contain a copyright notice

#ifndef THOR_SERVER__ACTION__DETAIL__JOINT_TASK__TRAITS_HPP_
#define THOR_SERVER__ACTION__DETAIL__JOINT_TASK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "thor_server/action/detail/joint_task__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace thor_server
{

namespace action
{

inline void to_flow_style_yaml(
  const JointTask_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint1_deg
  {
    out << "joint1_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.joint1_deg, out);
    out << ", ";
  }

  // member: joint2_deg
  {
    out << "joint2_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.joint2_deg, out);
    out << ", ";
  }

  // member: joint3_deg
  {
    out << "joint3_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.joint3_deg, out);
    out << ", ";
  }

  // member: joint4_deg
  {
    out << "joint4_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.joint4_deg, out);
    out << ", ";
  }

  // member: joint5_deg
  {
    out << "joint5_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.joint5_deg, out);
    out << ", ";
  }

  // member: joint6_deg
  {
    out << "joint6_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.joint6_deg, out);
    out << ", ";
  }

  // member: gripper_joint_deg
  {
    out << "gripper_joint_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_joint_deg, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointTask_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joint1_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint1_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.joint1_deg, out);
    out << "\n";
  }

  // member: joint2_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint2_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.joint2_deg, out);
    out << "\n";
  }

  // member: joint3_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint3_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.joint3_deg, out);
    out << "\n";
  }

  // member: joint4_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint4_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.joint4_deg, out);
    out << "\n";
  }

  // member: joint5_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint5_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.joint5_deg, out);
    out << "\n";
  }

  // member: joint6_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint6_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.joint6_deg, out);
    out << "\n";
  }

  // member: gripper_joint_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gripper_joint_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_joint_deg, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointTask_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace thor_server

namespace rosidl_generator_traits
{

[[deprecated("use thor_server::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const thor_server::action::JointTask_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  thor_server::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thor_server::action::to_yaml() instead")]]
inline std::string to_yaml(const thor_server::action::JointTask_Goal & msg)
{
  return thor_server::action::to_yaml(msg);
}

template<>
inline const char * data_type<thor_server::action::JointTask_Goal>()
{
  return "thor_server::action::JointTask_Goal";
}

template<>
inline const char * name<thor_server::action::JointTask_Goal>()
{
  return "thor_server/action/JointTask_Goal";
}

template<>
struct has_fixed_size<thor_server::action::JointTask_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<thor_server::action::JointTask_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<thor_server::action::JointTask_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace thor_server
{

namespace action
{

inline void to_flow_style_yaml(
  const JointTask_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointTask_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointTask_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace thor_server

namespace rosidl_generator_traits
{

[[deprecated("use thor_server::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const thor_server::action::JointTask_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  thor_server::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thor_server::action::to_yaml() instead")]]
inline std::string to_yaml(const thor_server::action::JointTask_Result & msg)
{
  return thor_server::action::to_yaml(msg);
}

template<>
inline const char * data_type<thor_server::action::JointTask_Result>()
{
  return "thor_server::action::JointTask_Result";
}

template<>
inline const char * name<thor_server::action::JointTask_Result>()
{
  return "thor_server/action/JointTask_Result";
}

template<>
struct has_fixed_size<thor_server::action::JointTask_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<thor_server::action::JointTask_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<thor_server::action::JointTask_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace thor_server
{

namespace action
{

inline void to_flow_style_yaml(
  const JointTask_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: percentage
  {
    out << "percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.percentage, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointTask_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.percentage, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointTask_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace thor_server

namespace rosidl_generator_traits
{

[[deprecated("use thor_server::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const thor_server::action::JointTask_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  thor_server::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thor_server::action::to_yaml() instead")]]
inline std::string to_yaml(const thor_server::action::JointTask_Feedback & msg)
{
  return thor_server::action::to_yaml(msg);
}

template<>
inline const char * data_type<thor_server::action::JointTask_Feedback>()
{
  return "thor_server::action::JointTask_Feedback";
}

template<>
inline const char * name<thor_server::action::JointTask_Feedback>()
{
  return "thor_server/action/JointTask_Feedback";
}

template<>
struct has_fixed_size<thor_server::action::JointTask_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<thor_server::action::JointTask_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<thor_server::action::JointTask_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "thor_server/action/detail/joint_task__traits.hpp"

namespace thor_server
{

namespace action
{

inline void to_flow_style_yaml(
  const JointTask_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointTask_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointTask_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace thor_server

namespace rosidl_generator_traits
{

[[deprecated("use thor_server::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const thor_server::action::JointTask_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  thor_server::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thor_server::action::to_yaml() instead")]]
inline std::string to_yaml(const thor_server::action::JointTask_SendGoal_Request & msg)
{
  return thor_server::action::to_yaml(msg);
}

template<>
inline const char * data_type<thor_server::action::JointTask_SendGoal_Request>()
{
  return "thor_server::action::JointTask_SendGoal_Request";
}

template<>
inline const char * name<thor_server::action::JointTask_SendGoal_Request>()
{
  return "thor_server/action/JointTask_SendGoal_Request";
}

template<>
struct has_fixed_size<thor_server::action::JointTask_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<thor_server::action::JointTask_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<thor_server::action::JointTask_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<thor_server::action::JointTask_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<thor_server::action::JointTask_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace thor_server
{

namespace action
{

inline void to_flow_style_yaml(
  const JointTask_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointTask_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointTask_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace thor_server

namespace rosidl_generator_traits
{

[[deprecated("use thor_server::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const thor_server::action::JointTask_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  thor_server::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thor_server::action::to_yaml() instead")]]
inline std::string to_yaml(const thor_server::action::JointTask_SendGoal_Response & msg)
{
  return thor_server::action::to_yaml(msg);
}

template<>
inline const char * data_type<thor_server::action::JointTask_SendGoal_Response>()
{
  return "thor_server::action::JointTask_SendGoal_Response";
}

template<>
inline const char * name<thor_server::action::JointTask_SendGoal_Response>()
{
  return "thor_server/action/JointTask_SendGoal_Response";
}

template<>
struct has_fixed_size<thor_server::action::JointTask_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<thor_server::action::JointTask_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<thor_server::action::JointTask_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<thor_server::action::JointTask_SendGoal>()
{
  return "thor_server::action::JointTask_SendGoal";
}

template<>
inline const char * name<thor_server::action::JointTask_SendGoal>()
{
  return "thor_server/action/JointTask_SendGoal";
}

template<>
struct has_fixed_size<thor_server::action::JointTask_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<thor_server::action::JointTask_SendGoal_Request>::value &&
    has_fixed_size<thor_server::action::JointTask_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<thor_server::action::JointTask_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<thor_server::action::JointTask_SendGoal_Request>::value &&
    has_bounded_size<thor_server::action::JointTask_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<thor_server::action::JointTask_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<thor_server::action::JointTask_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<thor_server::action::JointTask_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace thor_server
{

namespace action
{

inline void to_flow_style_yaml(
  const JointTask_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointTask_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointTask_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace thor_server

namespace rosidl_generator_traits
{

[[deprecated("use thor_server::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const thor_server::action::JointTask_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  thor_server::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thor_server::action::to_yaml() instead")]]
inline std::string to_yaml(const thor_server::action::JointTask_GetResult_Request & msg)
{
  return thor_server::action::to_yaml(msg);
}

template<>
inline const char * data_type<thor_server::action::JointTask_GetResult_Request>()
{
  return "thor_server::action::JointTask_GetResult_Request";
}

template<>
inline const char * name<thor_server::action::JointTask_GetResult_Request>()
{
  return "thor_server/action/JointTask_GetResult_Request";
}

template<>
struct has_fixed_size<thor_server::action::JointTask_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<thor_server::action::JointTask_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<thor_server::action::JointTask_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "thor_server/action/detail/joint_task__traits.hpp"

namespace thor_server
{

namespace action
{

inline void to_flow_style_yaml(
  const JointTask_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointTask_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointTask_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace thor_server

namespace rosidl_generator_traits
{

[[deprecated("use thor_server::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const thor_server::action::JointTask_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  thor_server::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thor_server::action::to_yaml() instead")]]
inline std::string to_yaml(const thor_server::action::JointTask_GetResult_Response & msg)
{
  return thor_server::action::to_yaml(msg);
}

template<>
inline const char * data_type<thor_server::action::JointTask_GetResult_Response>()
{
  return "thor_server::action::JointTask_GetResult_Response";
}

template<>
inline const char * name<thor_server::action::JointTask_GetResult_Response>()
{
  return "thor_server/action/JointTask_GetResult_Response";
}

template<>
struct has_fixed_size<thor_server::action::JointTask_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<thor_server::action::JointTask_Result>::value> {};

template<>
struct has_bounded_size<thor_server::action::JointTask_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<thor_server::action::JointTask_Result>::value> {};

template<>
struct is_message<thor_server::action::JointTask_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<thor_server::action::JointTask_GetResult>()
{
  return "thor_server::action::JointTask_GetResult";
}

template<>
inline const char * name<thor_server::action::JointTask_GetResult>()
{
  return "thor_server/action/JointTask_GetResult";
}

template<>
struct has_fixed_size<thor_server::action::JointTask_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<thor_server::action::JointTask_GetResult_Request>::value &&
    has_fixed_size<thor_server::action::JointTask_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<thor_server::action::JointTask_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<thor_server::action::JointTask_GetResult_Request>::value &&
    has_bounded_size<thor_server::action::JointTask_GetResult_Response>::value
  >
{
};

template<>
struct is_service<thor_server::action::JointTask_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<thor_server::action::JointTask_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<thor_server::action::JointTask_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "thor_server/action/detail/joint_task__traits.hpp"

namespace thor_server
{

namespace action
{

inline void to_flow_style_yaml(
  const JointTask_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointTask_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointTask_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace thor_server

namespace rosidl_generator_traits
{

[[deprecated("use thor_server::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const thor_server::action::JointTask_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  thor_server::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thor_server::action::to_yaml() instead")]]
inline std::string to_yaml(const thor_server::action::JointTask_FeedbackMessage & msg)
{
  return thor_server::action::to_yaml(msg);
}

template<>
inline const char * data_type<thor_server::action::JointTask_FeedbackMessage>()
{
  return "thor_server::action::JointTask_FeedbackMessage";
}

template<>
inline const char * name<thor_server::action::JointTask_FeedbackMessage>()
{
  return "thor_server/action/JointTask_FeedbackMessage";
}

template<>
struct has_fixed_size<thor_server::action::JointTask_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<thor_server::action::JointTask_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<thor_server::action::JointTask_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<thor_server::action::JointTask_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<thor_server::action::JointTask_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<thor_server::action::JointTask>
  : std::true_type
{
};

template<>
struct is_action_goal<thor_server::action::JointTask_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<thor_server::action::JointTask_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<thor_server::action::JointTask_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // THOR_SERVER__ACTION__DETAIL__JOINT_TASK__TRAITS_HPP_
