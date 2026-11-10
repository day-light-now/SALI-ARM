// SPDX-License-Identifier: Apache-2.0
// Standalone check of the four-axis gravity feedforward model (no ROS needed).
#include "slaihand_hardware/gravity_ff.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace {
int failures = 0;

void check(bool condition, const char* what) {
  if (!condition) {
    std::printf("FAIL %s\n", what);
    ++failures;
  }
}

bool near(double a, double b, double tolerance) { return std::abs(a - b) <= tolerance; }

// base -> link1 --J1(+y)--> link2 --J2(+z)--> link3 --J3(+z)--> link4 --J4(+z)--> link5
// Only link2 carries mass (2 kg at (0.5, 0, 0)), so only J1 sees gravity.
std::string pendulum_text(double gravity_z = -9.8, double j1_offset = 0.0) {
  const std::string g = "G:0:0:" + std::to_string(gravity_z);
  const std::string links = "|L:link1:2:" + std::to_string(0.5) + ":0:0";
  auto joint = [](const std::string& name, const std::string& parent, const std::string& child,
                  const std::string& axis, double offset) {
    char buffer[512];
    std::snprintf(buffer, sizeof(buffer),
                  "|J:%s:%s:%s:0:0:0:0:0:0:%s:%.17g:-3.3:3.3:r", name.c_str(), parent.c_str(),
                  child.c_str(), axis.c_str(), offset);
    return std::string(buffer);
  };
  return g + links + joint("slaihand_joint1", "base_link", "link1", "0:1:0", j1_offset) +
         joint("slaihand_joint2", "link1", "link2", "0:0:1", 0.0) +
         joint("slaihand_joint3", "link2", "link3", "0:0:1", 0.0) +
         joint("slaihand_joint4", "link3", "link4", "0:0:1", 0.0);
}

void expect_throw(const std::string& text, const char* what) {
  try {
    slaihand_hardware::GravityFfModel::parse(text);
  } catch (const std::invalid_argument&) {
    return;
  }
  std::printf("FAIL no throw: %s\n", what);
  ++failures;
}

void test_pendulum() {
  const auto model = slaihand_hardware::GravityFfModel::parse(pendulum_text());
  check(model.joint_names().size() == 4, "four movable joints");
  check(near(model.total_mass(), 2.0, 1e-12), "total mass");
  for (const double q1 : {0.0, 0.3, 1.2}) {
    const std::array<double, 4> q{q1, 0.4, -0.2, 0.1};
    const auto torque = model.torque(q);
    const double expected = -2.0 * 9.8 * 0.5 * std::cos(q1);
    check(near(torque[0], expected, 1e-12), "pendulum holding torque");
    for (std::size_t index = 1; index < torque.size(); ++index)
      check(near(torque[index], 0.0, 1e-12), "downstream joints carry no gravity torque");
  }
}

void test_energy_difference() {
  const auto model = slaihand_hardware::GravityFfModel::parse(pendulum_text(-9.80665, 0.0));
  const std::array<double, 4> q{0.37, -0.4, 0.2, 0.1};
  const auto torque = model.torque(q);
  const double h = 1e-6;
  for (std::size_t index = 0; index < 4; ++index) {
    auto forward = q;
    auto backward = q;
    forward[index] += h;
    backward[index] -= h;
    const double finite = (model.potential_energy(forward) - model.potential_energy(backward)) / (2 * h);
    check(near(torque[index], finite, 1e-6), "torque equals the potential-energy derivative");
  }
}

void test_offset_rotates_the_body() {
  const auto plain = slaihand_hardware::GravityFfModel::parse(pendulum_text(-9.8, 0.0));
  const auto rotated = slaihand_hardware::GravityFfModel::parse(pendulum_text(-9.8, M_PI / 2));
  const std::array<double, 4> q{0.2, 0.0, 0.0, 0.0};
  const double a = plain.torque(q)[0];
  const double b = rotated.torque(q)[0];
  // A fixed +pi/2 body rotation turns the arm's cos term into a sin term.
  check(!near(a, b, 1e-9), "fixed joint offset changes the gravity torque");
  const double expected = -2.0 * 9.8 * 0.5 * std::cos(0.2 + M_PI / 2);
  check(near(b, expected, 1e-12), "fixed joint offset applied exactly once");
}

void test_roots_and_validation() {
  const auto model = slaihand_hardware::GravityFfModel::parse(pendulum_text());
  check(!model.empty(), "model is not empty");
  check(near(model.gravity()[2], -9.8, 1e-12), "gravity vector");
  bool threw = false;
  try {
    model.torque({0.0, 0.0, 0.0, 9.0});
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  check(threw, "joint outside URDF limits is rejected");
  expect_throw("G:0:0:-9.8", "single gravity record is not a model");
  expect_throw("L:link1:2:0:0:0", "model without joints is rejected");
  std::string three = pendulum_text();
  three.erase(three.find("|J:slaihand_joint4"), std::string::npos);
  expect_throw(three, "three movable joints are rejected");
  expect_throw(pendulum_text() + "|X:1", "unknown record is rejected");
  expect_throw("G:0:0:-9.8|L:link2:0:0.5:0:0", "zero mass is rejected");
  expect_throw("G:0:0:-9.8|L:link2:2:nan:0:0", "non-finite COM is rejected");
}

void test_disconnected_tree() {
  std::string text = pendulum_text();
  const std::string parent = "J:slaihand_joint2:link1:link2";
  const std::string broken = "J:slaihand_joint2:orphan:link2";
  const auto position = text.find(parent);
  text.replace(position, parent.size(), broken);
  expect_throw(text, "disconnected kinematic tree is rejected");
}
void test_identified_parameters() {
  std::string text = pendulum_text();
  const auto first = text.find("|L:");
  const auto next = text.find("|J:");
  text.replace(first, next-first, "|B:link1:0:1:0:0");
  const auto effective = slaihand_hardware::GravityFfModel::parse(text);
  const auto physical = slaihand_hardware::GravityFfModel::parse(pendulum_text());
  for (double angle : {-.7, 0., .8}) {
    const std::array<double,4> q{angle,.3,-.2,.1};
    const auto a=effective.torque(q), b=physical.torque(q);
    for (std::size_t i=0;i<4;++i) check(near(a[i],b[i],1e-12), "identified first moment parity");
    check(near(effective.potential_energy(q),physical.potential_energy(q),1e-12), "identified energy parity");
  }
  bool mass_unknown=false;
  try { effective.total_mass(); } catch (const std::logic_error&) { mass_unknown=true; }
  check(mass_unknown, "effective coefficients do not claim physical total mass");
  expect_throw(text+"|B:link1:0:1:0:0", "duplicate effective link rejected");
  text.replace(text.find("B:link1:0:1:0:0"),std::string("B:link1:0:1:0:0").size(),"B:link1:0:nan:0:0");
  expect_throw(text, "nonfinite effective first moment rejected");
}
}  // namespace

int main() {
  test_pendulum();
  test_identified_parameters();
  test_energy_difference();
  test_offset_rotates_the_body();
  test_roots_and_validation();
  test_disconnected_tree();
  if (failures != 0) {
    std::printf("FAILED %d\n", failures);
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
