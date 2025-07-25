// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from thor_server:action/PoseTask.idl
// generated code does not contain a copyright notice
#include "thor_server/action/detail/pose_task__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
thor_server__action__PoseTask_Goal__init(thor_server__action__PoseTask_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  // roll
  // pitch
  // yaw
  return true;
}

void
thor_server__action__PoseTask_Goal__fini(thor_server__action__PoseTask_Goal * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
  // roll
  // pitch
  // yaw
}

bool
thor_server__action__PoseTask_Goal__are_equal(const thor_server__action__PoseTask_Goal * lhs, const thor_server__action__PoseTask_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
thor_server__action__PoseTask_Goal__copy(
  const thor_server__action__PoseTask_Goal * input,
  thor_server__action__PoseTask_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  return true;
}

thor_server__action__PoseTask_Goal *
thor_server__action__PoseTask_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_Goal * msg = (thor_server__action__PoseTask_Goal *)allocator.allocate(sizeof(thor_server__action__PoseTask_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(thor_server__action__PoseTask_Goal));
  bool success = thor_server__action__PoseTask_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
thor_server__action__PoseTask_Goal__destroy(thor_server__action__PoseTask_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    thor_server__action__PoseTask_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
thor_server__action__PoseTask_Goal__Sequence__init(thor_server__action__PoseTask_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_Goal * data = NULL;

  if (size) {
    data = (thor_server__action__PoseTask_Goal *)allocator.zero_allocate(size, sizeof(thor_server__action__PoseTask_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = thor_server__action__PoseTask_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        thor_server__action__PoseTask_Goal__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
thor_server__action__PoseTask_Goal__Sequence__fini(thor_server__action__PoseTask_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      thor_server__action__PoseTask_Goal__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

thor_server__action__PoseTask_Goal__Sequence *
thor_server__action__PoseTask_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_Goal__Sequence * array = (thor_server__action__PoseTask_Goal__Sequence *)allocator.allocate(sizeof(thor_server__action__PoseTask_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = thor_server__action__PoseTask_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
thor_server__action__PoseTask_Goal__Sequence__destroy(thor_server__action__PoseTask_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    thor_server__action__PoseTask_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
thor_server__action__PoseTask_Goal__Sequence__are_equal(const thor_server__action__PoseTask_Goal__Sequence * lhs, const thor_server__action__PoseTask_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!thor_server__action__PoseTask_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
thor_server__action__PoseTask_Goal__Sequence__copy(
  const thor_server__action__PoseTask_Goal__Sequence * input,
  thor_server__action__PoseTask_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(thor_server__action__PoseTask_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    thor_server__action__PoseTask_Goal * data =
      (thor_server__action__PoseTask_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!thor_server__action__PoseTask_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          thor_server__action__PoseTask_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!thor_server__action__PoseTask_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
thor_server__action__PoseTask_Result__init(thor_server__action__PoseTask_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
thor_server__action__PoseTask_Result__fini(thor_server__action__PoseTask_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
thor_server__action__PoseTask_Result__are_equal(const thor_server__action__PoseTask_Result * lhs, const thor_server__action__PoseTask_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
thor_server__action__PoseTask_Result__copy(
  const thor_server__action__PoseTask_Result * input,
  thor_server__action__PoseTask_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

thor_server__action__PoseTask_Result *
thor_server__action__PoseTask_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_Result * msg = (thor_server__action__PoseTask_Result *)allocator.allocate(sizeof(thor_server__action__PoseTask_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(thor_server__action__PoseTask_Result));
  bool success = thor_server__action__PoseTask_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
thor_server__action__PoseTask_Result__destroy(thor_server__action__PoseTask_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    thor_server__action__PoseTask_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
thor_server__action__PoseTask_Result__Sequence__init(thor_server__action__PoseTask_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_Result * data = NULL;

  if (size) {
    data = (thor_server__action__PoseTask_Result *)allocator.zero_allocate(size, sizeof(thor_server__action__PoseTask_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = thor_server__action__PoseTask_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        thor_server__action__PoseTask_Result__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
thor_server__action__PoseTask_Result__Sequence__fini(thor_server__action__PoseTask_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      thor_server__action__PoseTask_Result__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

thor_server__action__PoseTask_Result__Sequence *
thor_server__action__PoseTask_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_Result__Sequence * array = (thor_server__action__PoseTask_Result__Sequence *)allocator.allocate(sizeof(thor_server__action__PoseTask_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = thor_server__action__PoseTask_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
thor_server__action__PoseTask_Result__Sequence__destroy(thor_server__action__PoseTask_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    thor_server__action__PoseTask_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
thor_server__action__PoseTask_Result__Sequence__are_equal(const thor_server__action__PoseTask_Result__Sequence * lhs, const thor_server__action__PoseTask_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!thor_server__action__PoseTask_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
thor_server__action__PoseTask_Result__Sequence__copy(
  const thor_server__action__PoseTask_Result__Sequence * input,
  thor_server__action__PoseTask_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(thor_server__action__PoseTask_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    thor_server__action__PoseTask_Result * data =
      (thor_server__action__PoseTask_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!thor_server__action__PoseTask_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          thor_server__action__PoseTask_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!thor_server__action__PoseTask_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
thor_server__action__PoseTask_Feedback__init(thor_server__action__PoseTask_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // percentage
  return true;
}

void
thor_server__action__PoseTask_Feedback__fini(thor_server__action__PoseTask_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // percentage
}

bool
thor_server__action__PoseTask_Feedback__are_equal(const thor_server__action__PoseTask_Feedback * lhs, const thor_server__action__PoseTask_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // percentage
  if (lhs->percentage != rhs->percentage) {
    return false;
  }
  return true;
}

bool
thor_server__action__PoseTask_Feedback__copy(
  const thor_server__action__PoseTask_Feedback * input,
  thor_server__action__PoseTask_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // percentage
  output->percentage = input->percentage;
  return true;
}

thor_server__action__PoseTask_Feedback *
thor_server__action__PoseTask_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_Feedback * msg = (thor_server__action__PoseTask_Feedback *)allocator.allocate(sizeof(thor_server__action__PoseTask_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(thor_server__action__PoseTask_Feedback));
  bool success = thor_server__action__PoseTask_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
thor_server__action__PoseTask_Feedback__destroy(thor_server__action__PoseTask_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    thor_server__action__PoseTask_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
thor_server__action__PoseTask_Feedback__Sequence__init(thor_server__action__PoseTask_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_Feedback * data = NULL;

  if (size) {
    data = (thor_server__action__PoseTask_Feedback *)allocator.zero_allocate(size, sizeof(thor_server__action__PoseTask_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = thor_server__action__PoseTask_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        thor_server__action__PoseTask_Feedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
thor_server__action__PoseTask_Feedback__Sequence__fini(thor_server__action__PoseTask_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      thor_server__action__PoseTask_Feedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

thor_server__action__PoseTask_Feedback__Sequence *
thor_server__action__PoseTask_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_Feedback__Sequence * array = (thor_server__action__PoseTask_Feedback__Sequence *)allocator.allocate(sizeof(thor_server__action__PoseTask_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = thor_server__action__PoseTask_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
thor_server__action__PoseTask_Feedback__Sequence__destroy(thor_server__action__PoseTask_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    thor_server__action__PoseTask_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
thor_server__action__PoseTask_Feedback__Sequence__are_equal(const thor_server__action__PoseTask_Feedback__Sequence * lhs, const thor_server__action__PoseTask_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!thor_server__action__PoseTask_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
thor_server__action__PoseTask_Feedback__Sequence__copy(
  const thor_server__action__PoseTask_Feedback__Sequence * input,
  thor_server__action__PoseTask_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(thor_server__action__PoseTask_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    thor_server__action__PoseTask_Feedback * data =
      (thor_server__action__PoseTask_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!thor_server__action__PoseTask_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          thor_server__action__PoseTask_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!thor_server__action__PoseTask_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "thor_server/action/detail/pose_task__functions.h"

bool
thor_server__action__PoseTask_SendGoal_Request__init(thor_server__action__PoseTask_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    thor_server__action__PoseTask_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!thor_server__action__PoseTask_Goal__init(&msg->goal)) {
    thor_server__action__PoseTask_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
thor_server__action__PoseTask_SendGoal_Request__fini(thor_server__action__PoseTask_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  thor_server__action__PoseTask_Goal__fini(&msg->goal);
}

bool
thor_server__action__PoseTask_SendGoal_Request__are_equal(const thor_server__action__PoseTask_SendGoal_Request * lhs, const thor_server__action__PoseTask_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!thor_server__action__PoseTask_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
thor_server__action__PoseTask_SendGoal_Request__copy(
  const thor_server__action__PoseTask_SendGoal_Request * input,
  thor_server__action__PoseTask_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!thor_server__action__PoseTask_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

thor_server__action__PoseTask_SendGoal_Request *
thor_server__action__PoseTask_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_SendGoal_Request * msg = (thor_server__action__PoseTask_SendGoal_Request *)allocator.allocate(sizeof(thor_server__action__PoseTask_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(thor_server__action__PoseTask_SendGoal_Request));
  bool success = thor_server__action__PoseTask_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
thor_server__action__PoseTask_SendGoal_Request__destroy(thor_server__action__PoseTask_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    thor_server__action__PoseTask_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
thor_server__action__PoseTask_SendGoal_Request__Sequence__init(thor_server__action__PoseTask_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_SendGoal_Request * data = NULL;

  if (size) {
    data = (thor_server__action__PoseTask_SendGoal_Request *)allocator.zero_allocate(size, sizeof(thor_server__action__PoseTask_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = thor_server__action__PoseTask_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        thor_server__action__PoseTask_SendGoal_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
thor_server__action__PoseTask_SendGoal_Request__Sequence__fini(thor_server__action__PoseTask_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      thor_server__action__PoseTask_SendGoal_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

thor_server__action__PoseTask_SendGoal_Request__Sequence *
thor_server__action__PoseTask_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_SendGoal_Request__Sequence * array = (thor_server__action__PoseTask_SendGoal_Request__Sequence *)allocator.allocate(sizeof(thor_server__action__PoseTask_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = thor_server__action__PoseTask_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
thor_server__action__PoseTask_SendGoal_Request__Sequence__destroy(thor_server__action__PoseTask_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    thor_server__action__PoseTask_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
thor_server__action__PoseTask_SendGoal_Request__Sequence__are_equal(const thor_server__action__PoseTask_SendGoal_Request__Sequence * lhs, const thor_server__action__PoseTask_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!thor_server__action__PoseTask_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
thor_server__action__PoseTask_SendGoal_Request__Sequence__copy(
  const thor_server__action__PoseTask_SendGoal_Request__Sequence * input,
  thor_server__action__PoseTask_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(thor_server__action__PoseTask_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    thor_server__action__PoseTask_SendGoal_Request * data =
      (thor_server__action__PoseTask_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!thor_server__action__PoseTask_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          thor_server__action__PoseTask_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!thor_server__action__PoseTask_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
thor_server__action__PoseTask_SendGoal_Response__init(thor_server__action__PoseTask_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    thor_server__action__PoseTask_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
thor_server__action__PoseTask_SendGoal_Response__fini(thor_server__action__PoseTask_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
thor_server__action__PoseTask_SendGoal_Response__are_equal(const thor_server__action__PoseTask_SendGoal_Response * lhs, const thor_server__action__PoseTask_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
thor_server__action__PoseTask_SendGoal_Response__copy(
  const thor_server__action__PoseTask_SendGoal_Response * input,
  thor_server__action__PoseTask_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

thor_server__action__PoseTask_SendGoal_Response *
thor_server__action__PoseTask_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_SendGoal_Response * msg = (thor_server__action__PoseTask_SendGoal_Response *)allocator.allocate(sizeof(thor_server__action__PoseTask_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(thor_server__action__PoseTask_SendGoal_Response));
  bool success = thor_server__action__PoseTask_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
thor_server__action__PoseTask_SendGoal_Response__destroy(thor_server__action__PoseTask_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    thor_server__action__PoseTask_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
thor_server__action__PoseTask_SendGoal_Response__Sequence__init(thor_server__action__PoseTask_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_SendGoal_Response * data = NULL;

  if (size) {
    data = (thor_server__action__PoseTask_SendGoal_Response *)allocator.zero_allocate(size, sizeof(thor_server__action__PoseTask_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = thor_server__action__PoseTask_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        thor_server__action__PoseTask_SendGoal_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
thor_server__action__PoseTask_SendGoal_Response__Sequence__fini(thor_server__action__PoseTask_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      thor_server__action__PoseTask_SendGoal_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

thor_server__action__PoseTask_SendGoal_Response__Sequence *
thor_server__action__PoseTask_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_SendGoal_Response__Sequence * array = (thor_server__action__PoseTask_SendGoal_Response__Sequence *)allocator.allocate(sizeof(thor_server__action__PoseTask_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = thor_server__action__PoseTask_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
thor_server__action__PoseTask_SendGoal_Response__Sequence__destroy(thor_server__action__PoseTask_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    thor_server__action__PoseTask_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
thor_server__action__PoseTask_SendGoal_Response__Sequence__are_equal(const thor_server__action__PoseTask_SendGoal_Response__Sequence * lhs, const thor_server__action__PoseTask_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!thor_server__action__PoseTask_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
thor_server__action__PoseTask_SendGoal_Response__Sequence__copy(
  const thor_server__action__PoseTask_SendGoal_Response__Sequence * input,
  thor_server__action__PoseTask_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(thor_server__action__PoseTask_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    thor_server__action__PoseTask_SendGoal_Response * data =
      (thor_server__action__PoseTask_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!thor_server__action__PoseTask_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          thor_server__action__PoseTask_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!thor_server__action__PoseTask_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
thor_server__action__PoseTask_GetResult_Request__init(thor_server__action__PoseTask_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    thor_server__action__PoseTask_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
thor_server__action__PoseTask_GetResult_Request__fini(thor_server__action__PoseTask_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
thor_server__action__PoseTask_GetResult_Request__are_equal(const thor_server__action__PoseTask_GetResult_Request * lhs, const thor_server__action__PoseTask_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
thor_server__action__PoseTask_GetResult_Request__copy(
  const thor_server__action__PoseTask_GetResult_Request * input,
  thor_server__action__PoseTask_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

thor_server__action__PoseTask_GetResult_Request *
thor_server__action__PoseTask_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_GetResult_Request * msg = (thor_server__action__PoseTask_GetResult_Request *)allocator.allocate(sizeof(thor_server__action__PoseTask_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(thor_server__action__PoseTask_GetResult_Request));
  bool success = thor_server__action__PoseTask_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
thor_server__action__PoseTask_GetResult_Request__destroy(thor_server__action__PoseTask_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    thor_server__action__PoseTask_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
thor_server__action__PoseTask_GetResult_Request__Sequence__init(thor_server__action__PoseTask_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_GetResult_Request * data = NULL;

  if (size) {
    data = (thor_server__action__PoseTask_GetResult_Request *)allocator.zero_allocate(size, sizeof(thor_server__action__PoseTask_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = thor_server__action__PoseTask_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        thor_server__action__PoseTask_GetResult_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
thor_server__action__PoseTask_GetResult_Request__Sequence__fini(thor_server__action__PoseTask_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      thor_server__action__PoseTask_GetResult_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

thor_server__action__PoseTask_GetResult_Request__Sequence *
thor_server__action__PoseTask_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_GetResult_Request__Sequence * array = (thor_server__action__PoseTask_GetResult_Request__Sequence *)allocator.allocate(sizeof(thor_server__action__PoseTask_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = thor_server__action__PoseTask_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
thor_server__action__PoseTask_GetResult_Request__Sequence__destroy(thor_server__action__PoseTask_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    thor_server__action__PoseTask_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
thor_server__action__PoseTask_GetResult_Request__Sequence__are_equal(const thor_server__action__PoseTask_GetResult_Request__Sequence * lhs, const thor_server__action__PoseTask_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!thor_server__action__PoseTask_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
thor_server__action__PoseTask_GetResult_Request__Sequence__copy(
  const thor_server__action__PoseTask_GetResult_Request__Sequence * input,
  thor_server__action__PoseTask_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(thor_server__action__PoseTask_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    thor_server__action__PoseTask_GetResult_Request * data =
      (thor_server__action__PoseTask_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!thor_server__action__PoseTask_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          thor_server__action__PoseTask_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!thor_server__action__PoseTask_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "thor_server/action/detail/pose_task__functions.h"

bool
thor_server__action__PoseTask_GetResult_Response__init(thor_server__action__PoseTask_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!thor_server__action__PoseTask_Result__init(&msg->result)) {
    thor_server__action__PoseTask_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
thor_server__action__PoseTask_GetResult_Response__fini(thor_server__action__PoseTask_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  thor_server__action__PoseTask_Result__fini(&msg->result);
}

bool
thor_server__action__PoseTask_GetResult_Response__are_equal(const thor_server__action__PoseTask_GetResult_Response * lhs, const thor_server__action__PoseTask_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!thor_server__action__PoseTask_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
thor_server__action__PoseTask_GetResult_Response__copy(
  const thor_server__action__PoseTask_GetResult_Response * input,
  thor_server__action__PoseTask_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!thor_server__action__PoseTask_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

thor_server__action__PoseTask_GetResult_Response *
thor_server__action__PoseTask_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_GetResult_Response * msg = (thor_server__action__PoseTask_GetResult_Response *)allocator.allocate(sizeof(thor_server__action__PoseTask_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(thor_server__action__PoseTask_GetResult_Response));
  bool success = thor_server__action__PoseTask_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
thor_server__action__PoseTask_GetResult_Response__destroy(thor_server__action__PoseTask_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    thor_server__action__PoseTask_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
thor_server__action__PoseTask_GetResult_Response__Sequence__init(thor_server__action__PoseTask_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_GetResult_Response * data = NULL;

  if (size) {
    data = (thor_server__action__PoseTask_GetResult_Response *)allocator.zero_allocate(size, sizeof(thor_server__action__PoseTask_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = thor_server__action__PoseTask_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        thor_server__action__PoseTask_GetResult_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
thor_server__action__PoseTask_GetResult_Response__Sequence__fini(thor_server__action__PoseTask_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      thor_server__action__PoseTask_GetResult_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

thor_server__action__PoseTask_GetResult_Response__Sequence *
thor_server__action__PoseTask_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_GetResult_Response__Sequence * array = (thor_server__action__PoseTask_GetResult_Response__Sequence *)allocator.allocate(sizeof(thor_server__action__PoseTask_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = thor_server__action__PoseTask_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
thor_server__action__PoseTask_GetResult_Response__Sequence__destroy(thor_server__action__PoseTask_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    thor_server__action__PoseTask_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
thor_server__action__PoseTask_GetResult_Response__Sequence__are_equal(const thor_server__action__PoseTask_GetResult_Response__Sequence * lhs, const thor_server__action__PoseTask_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!thor_server__action__PoseTask_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
thor_server__action__PoseTask_GetResult_Response__Sequence__copy(
  const thor_server__action__PoseTask_GetResult_Response__Sequence * input,
  thor_server__action__PoseTask_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(thor_server__action__PoseTask_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    thor_server__action__PoseTask_GetResult_Response * data =
      (thor_server__action__PoseTask_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!thor_server__action__PoseTask_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          thor_server__action__PoseTask_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!thor_server__action__PoseTask_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "thor_server/action/detail/pose_task__functions.h"

bool
thor_server__action__PoseTask_FeedbackMessage__init(thor_server__action__PoseTask_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    thor_server__action__PoseTask_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!thor_server__action__PoseTask_Feedback__init(&msg->feedback)) {
    thor_server__action__PoseTask_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
thor_server__action__PoseTask_FeedbackMessage__fini(thor_server__action__PoseTask_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  thor_server__action__PoseTask_Feedback__fini(&msg->feedback);
}

bool
thor_server__action__PoseTask_FeedbackMessage__are_equal(const thor_server__action__PoseTask_FeedbackMessage * lhs, const thor_server__action__PoseTask_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!thor_server__action__PoseTask_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
thor_server__action__PoseTask_FeedbackMessage__copy(
  const thor_server__action__PoseTask_FeedbackMessage * input,
  thor_server__action__PoseTask_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!thor_server__action__PoseTask_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

thor_server__action__PoseTask_FeedbackMessage *
thor_server__action__PoseTask_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_FeedbackMessage * msg = (thor_server__action__PoseTask_FeedbackMessage *)allocator.allocate(sizeof(thor_server__action__PoseTask_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(thor_server__action__PoseTask_FeedbackMessage));
  bool success = thor_server__action__PoseTask_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
thor_server__action__PoseTask_FeedbackMessage__destroy(thor_server__action__PoseTask_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    thor_server__action__PoseTask_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
thor_server__action__PoseTask_FeedbackMessage__Sequence__init(thor_server__action__PoseTask_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_FeedbackMessage * data = NULL;

  if (size) {
    data = (thor_server__action__PoseTask_FeedbackMessage *)allocator.zero_allocate(size, sizeof(thor_server__action__PoseTask_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = thor_server__action__PoseTask_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        thor_server__action__PoseTask_FeedbackMessage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
thor_server__action__PoseTask_FeedbackMessage__Sequence__fini(thor_server__action__PoseTask_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      thor_server__action__PoseTask_FeedbackMessage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

thor_server__action__PoseTask_FeedbackMessage__Sequence *
thor_server__action__PoseTask_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thor_server__action__PoseTask_FeedbackMessage__Sequence * array = (thor_server__action__PoseTask_FeedbackMessage__Sequence *)allocator.allocate(sizeof(thor_server__action__PoseTask_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = thor_server__action__PoseTask_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
thor_server__action__PoseTask_FeedbackMessage__Sequence__destroy(thor_server__action__PoseTask_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    thor_server__action__PoseTask_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
thor_server__action__PoseTask_FeedbackMessage__Sequence__are_equal(const thor_server__action__PoseTask_FeedbackMessage__Sequence * lhs, const thor_server__action__PoseTask_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!thor_server__action__PoseTask_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
thor_server__action__PoseTask_FeedbackMessage__Sequence__copy(
  const thor_server__action__PoseTask_FeedbackMessage__Sequence * input,
  thor_server__action__PoseTask_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(thor_server__action__PoseTask_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    thor_server__action__PoseTask_FeedbackMessage * data =
      (thor_server__action__PoseTask_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!thor_server__action__PoseTask_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          thor_server__action__PoseTask_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!thor_server__action__PoseTask_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
