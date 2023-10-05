// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_message:msg/ControlInputs.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGE__MSG__DETAIL__CONTROL_INPUTS__STRUCT_H_
#define CUSTOM_MESSAGE__MSG__DETAIL__CONTROL_INPUTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ControlInputs in the package custom_message.
typedef struct custom_message__msg__ControlInputs
{
  float throttle;
  float delta;
} custom_message__msg__ControlInputs;

// Struct for a sequence of custom_message__msg__ControlInputs.
typedef struct custom_message__msg__ControlInputs__Sequence
{
  custom_message__msg__ControlInputs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_message__msg__ControlInputs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__CONTROL_INPUTS__STRUCT_H_
