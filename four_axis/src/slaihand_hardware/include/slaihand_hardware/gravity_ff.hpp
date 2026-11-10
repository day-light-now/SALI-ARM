// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace slaihand_hardware {
// Minimal four-axis gravity model loaded from a compact text parameter that the
// launch generates from the canonical URDF plus the mass profile. Pure C++17,
// no ROS, no file access, no dynamics beyond gravity (q_dot = 0, q_ddot = 0).
struct GravityLink {
  std::string name;
  double mass{0};
  std::array<double, 3> com{};
  std::array<double, 3> first_moment{};
  bool effective_parameters{false};
};
struct GravityJoint {
  std::string name, parent, child;
  std::array<double, 3> xyz{}, rpy{}, axis{};
  double offset{0};
  bool revolute{false};
  double lower{0}, upper{0};
};

class GravityFfModel {
 public:
  // Text format (single line, '|' separates records, ':' separates fields):
  //   G:gx:gy:gz
  //   L:name:mass:comx:comy:comz
  //   B:name:effective_mass:effective_hx:effective_hy:effective_hz
  // B contains identified linear coefficients, not individual physical masses.
  //   J:name:parent:child:xyz_x:xyz_y:xyz_z:rpy_r:rpy_p:rpy_y:axis_x:axis_y:axis_z:
  //     offset:lower:upper:kind      (kind is 'r' for revolute or 'f' for fixed)
  // Throws std::invalid_argument on malformed or non-finite data.
  static GravityFfModel parse(const std::string& text);
  bool empty() const { return joints_.empty(); }
  const std::vector<std::string>& joint_names() const { return movable_names_; }
  const std::array<double, 3>& gravity() const { return gravity_; }
  double total_mass() const;
  // Gravity joint torque in the URDF model frame (dU/dq), ordered like
  // joint_names(). Throws when positions are non-finite or exceed limits.
  std::vector<double> torque(const std::array<double, 4>& q_model) const;
  double potential_energy(const std::array<double, 4>& q_model) const;
  std::size_t movable_index(const std::string& name) const;

 private:
  struct Frame {
    std::array<std::array<double, 3>, 3> rotation{};
    std::array<double, 3> position{};
  };
  void evaluate(const std::array<double, 4>& q_model, std::vector<double>& torque,
                double& energy) const;
  std::vector<GravityLink> links_;
  std::vector<GravityJoint> joints_;
  std::array<double, 3> gravity_{};
  std::vector<std::string> movable_names_;
  std::string root_;
};
}  // namespace slaihand_hardware
