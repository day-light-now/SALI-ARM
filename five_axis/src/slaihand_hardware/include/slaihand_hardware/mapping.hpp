// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <stdexcept>

namespace slaihand_hardware {
struct JointMapping {
  // Fake defaults only. Actual motor values are still null in hardware_mapping.yaml.
  double sign = 1.0;
  double zero = 0.0;
  void validate() const {
    if ((sign != 1.0 && sign != -1.0) || !std::isfinite(zero)) {
      throw std::invalid_argument("Mapping requires sign +/-1 and finite zero");
    }
  }
  double to_motor(double joint) const { return sign * joint + zero; }
  double to_joint(double motor) const { return sign * (motor - zero); }
};
}  // namespace slaihand_hardware
