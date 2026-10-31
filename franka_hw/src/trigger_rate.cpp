// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/trigger_rate.h>

#include <rclcpp/rclcpp.hpp>

namespace franka_hw {

TriggerRate::TriggerRate(double rate)
    : period_(1.0 / rate), time_stamp_(rclcpp::Clock(RCL_SYSTEM_TIME).now()) {}

bool TriggerRate::operator()() {
  rclcpp::Time now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  if ((now - time_stamp_).seconds() > period_) {
    time_stamp_ = now;
    return true;
  }
  return false;
}

}  // namespace franka_hw
