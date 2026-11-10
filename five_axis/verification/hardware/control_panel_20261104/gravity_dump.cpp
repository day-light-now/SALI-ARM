// SPDX-License-Identifier: Apache-2.0
// Dump four-axis gravity torques for cross-checking the C++ model against the
// Python production model. Reads the model text from argv[1], then reads lines of
// four radian angles from stdin and prints torque(4) + potential energy per line.
#include "slaihand_hardware/gravity_ff.hpp"

#include <array>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <gravity_model_text_file>\n", argv[0]);
    return 2;
  }
  std::ifstream stream(argv[1]);
  if (!stream) {
    std::fprintf(stderr, "cannot read model text\n");
    return 2;
  }
  std::stringstream buffer;
  buffer << stream.rdbuf();
  slaihand_hardware::GravityFfModel model;
  try {
    model = slaihand_hardware::GravityFfModel::parse(buffer.str());
  } catch (const std::exception& error) {
    std::fprintf(stderr, "model parse failed: %s\n", error.what());
    return 2;
  }
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    std::istringstream fields(line);
    std::array<double, 4> q{};
    for (double& value : q)
      if (!(fields >> value)) {
        std::fprintf(stderr, "bad angle line\n");
        return 2;
      }
    const auto torque = model.torque(q);
    std::printf("%.17g %.17g %.17g %.17g %.17g\n", torque[0], torque[1], torque[2], torque[3],
                model.potential_energy(q));
  }
  return 0;
}
