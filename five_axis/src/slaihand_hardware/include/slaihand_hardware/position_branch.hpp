// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <slaihand_transport/canusb_readonly.hpp>
#include <cmath>
#include <stdexcept>

namespace slaihand_hardware {
struct PositionBranch {
  slaihand_transport::Mapping mapping;
  int model_turns;
};
// Resolve once from a fresh, disabled startup pose. Both RX and native MIT TX
// must use the returned mapping throughout the connection. Never modulo in flight.
inline PositionBranch startup_position_branch(const slaihand_transport::Mapping& canonical,
                                              double raw,double lower,double upper,double pmax) {
  const double turn=2*std::acos(-1.);
  if(!std::isfinite(raw)||!std::isfinite(canonical.zero)||!std::isfinite(lower)||
     !std::isfinite(upper)||!std::isfinite(pmax)||pmax<=0||std::abs(raw)>pmax||
     (canonical.sign!=1&&canonical.sign!=-1)||lower>=upper||upper-lower>=turn)
    throw std::invalid_argument("Invalid or ambiguous startup position sector");
  const double q=canonical.sign*(raw-canonical.zero);
  const double first=std::ceil((q-upper)/turn),last=std::floor((q-lower)/turn);
  if(first!=last||std::abs(first)>100)
    throw std::invalid_argument("No unique startup position branch inside URDF limits");
  auto effective=canonical;
  effective.zero+=canonical.sign*turn*first;
  const double raw_lower=effective.zero+canonical.sign*lower;
  const double raw_upper=effective.zero+canonical.sign*upper;
  if(std::abs(raw_lower)>pmax||std::abs(raw_upper)>pmax)
    throw std::invalid_argument("Startup position branch cannot represent the full URDF sector");
  return {effective,static_cast<int>(first)};
}
}
