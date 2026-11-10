// SPDX-License-Identifier: Apache-2.0
#include "slaihand_hardware/gravity_ff.hpp"

#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace slaihand_hardware {
namespace {
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;

double strict_number(const std::string& text, const char* what) {
  std::size_t used = 0;
  double value = 0;
  try {
    value = std::stod(text, &used);
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string("Invalid gravity model number: ") + what);
  }
  if (used != text.size() || !std::isfinite(value))
    throw std::invalid_argument(std::string("Invalid gravity model number: ") + what);
  return value;
}

std::vector<std::string> split(const std::string& text, char separator) {
  std::vector<std::string> parts;
  std::string current;
  for (const char character : text) {
    if (character == separator) {
      parts.push_back(current);
      current.clear();
    } else {
      current.push_back(character);
    }
  }
  parts.push_back(current);
  return parts;
}

Vec3 parse_vec(const std::vector<std::string>& fields, std::size_t first, const char* what) {
  return {strict_number(fields.at(first), what), strict_number(fields.at(first + 1), what),
          strict_number(fields.at(first + 2), what)};
}

Mat3 identity() { return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}; }

Mat3 multiply(const Mat3& a, const Mat3& b) {
  Mat3 out{};
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j) {
      double sum = 0;
      for (std::size_t k = 0; k < 3; ++k) sum += a[i][k] * b[k][j];
      out[i][j] = sum;
    }
  return out;
}

Vec3 rotate(const Mat3& a, const Vec3& v) {
  Vec3 out{};
  for (std::size_t i = 0; i < 3; ++i)
    out[i] = a[i][0] * v[0] + a[i][1] * v[1] + a[i][2] * v[2];
  return out;
}

Mat3 rotation_xyz(const Vec3& rpy) {
  const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
  const double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
  return {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
           {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
           {-sp, cp * sr, cp * cr}}};
}

Mat3 rotation_axis(const Vec3& axis, double angle) {
  double norm = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
  if (!std::isfinite(norm) || norm <= 0) throw std::invalid_argument("Invalid gravity model axis");
  const double x = axis[0] / norm, y = axis[1] / norm, z = axis[2] / norm;
  const double c = std::cos(angle), s = std::sin(angle), t = 1.0 - c;
  return {{{t * x * x + c, t * x * y - s * z, t * x * z + s * y},
           {t * x * y + s * z, t * y * y + c, t * y * z - s * x},
           {t * x * z - s * y, t * y * z + s * x, t * z * z + c}}};
}

double dot(const Vec3& a, const Vec3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
Vec3 add(const Vec3& a, const Vec3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
Vec3 sub(const Vec3& a, const Vec3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }

struct ChainEntry {
  std::size_t index;
  Vec3 pivot;
  Vec3 axis;
};
}  // namespace

GravityFfModel GravityFfModel::parse(const std::string& text) {
  GravityFfModel model;
  bool has_gravity = false;
  std::set<std::string> movable;
  for (const auto& record : split(text, '|')) {
    if (record.empty()) continue;
    const auto fields = split(record, ':');
    const std::string& kind = fields.at(0);
    if (kind == "G") {
      if (fields.size() != 4) throw std::invalid_argument("Invalid gravity record");
      model.gravity_ = parse_vec(fields, 1, "gravity");
      has_gravity = true;
    } else if (kind == "L" || kind == "B") {
      if (fields.size() != 6) throw std::invalid_argument("Invalid link record");
      GravityLink link;
      link.name = fields[1];
      if (link.name.empty()) throw std::invalid_argument("Empty link name in gravity model");
      for (const auto& existing : model.links_)
        if (existing.name == link.name) throw std::invalid_argument("Duplicate gravity link record");
      link.mass = strict_number(fields[2], "mass");
      link.effective_parameters = kind == "B";
      if (link.effective_parameters) {
        link.first_moment = parse_vec(fields, 3, "effective first moment");
      } else {
        if (link.mass <= 0) throw std::invalid_argument("Gravity model mass must be positive");
        link.com = parse_vec(fields, 3, "com");
      }
      model.links_.push_back(link);
    } else if (kind == "J") {
      if (fields.size() != 17) throw std::invalid_argument("Invalid joint record");
      GravityJoint joint;
      joint.name = fields[1];
      joint.parent = fields[2];
      joint.child = fields[3];
      if (joint.name.empty() || joint.parent.empty() || joint.child.empty())
        throw std::invalid_argument("Empty joint fields in gravity model");
      joint.xyz = parse_vec(fields, 4, "xyz");
      joint.rpy = parse_vec(fields, 7, "rpy");
      joint.axis = parse_vec(fields, 10, "axis");
      joint.offset = strict_number(fields[13], "offset");
      joint.lower = strict_number(fields[14], "lower");
      joint.upper = strict_number(fields[15], "upper");
      const std::string& kind_text = fields[16];
      if (kind_text != "r" && kind_text != "f")
        throw std::invalid_argument("Invalid joint kind in gravity model");
      joint.revolute = kind_text == "r";
      if (joint.revolute) {
        const double norm = std::sqrt(dot(joint.axis, joint.axis));
        if (!std::isfinite(norm) || norm <= 0) throw std::invalid_argument("Invalid joint axis");
        for (auto& value : joint.axis) value /= norm;
        if (!(joint.lower < joint.upper)) throw std::invalid_argument("Invalid gravity joint limits");
        if (!movable.insert(joint.name).second)
          throw std::invalid_argument("Duplicate movable joint in gravity model");
        model.movable_names_.push_back(joint.name);
      }
      model.joints_.push_back(joint);
    } else {
      throw std::invalid_argument("Unknown gravity model record");
    }
  }
  if (!has_gravity) throw std::invalid_argument("Gravity model has no gravity vector");
  if (model.movable_names_.size() != 4)
    throw std::invalid_argument("Gravity model must contain four movable joints");
  std::set<std::string> children;
  std::set<std::string> link_names;
  for (const auto& joint : model.joints_) {
    children.insert(joint.child);
    link_names.insert(joint.parent);
    link_names.insert(joint.child);
  }
  for (const auto& link : model.links_) link_names.insert(link.name);
  for (const auto& name : link_names) {
    if (children.count(name) == 0) {
      if (!model.root_.empty()) throw std::invalid_argument("Gravity model has multiple roots");
      model.root_ = name;
    }
  }
  if (model.root_.empty()) throw std::invalid_argument("Gravity model has no root link");
  return model;
}

std::size_t GravityFfModel::movable_index(const std::string& name) const {
  for (std::size_t index = 0; index < movable_names_.size(); ++index)
    if (movable_names_[index] == name) return index;
  throw std::invalid_argument("Unknown gravity model joint: " + name);
}

double GravityFfModel::total_mass() const {
  double total = 0;
  for (const auto& link : links_) {
    if (link.effective_parameters) throw std::logic_error("Identified base parameters do not determine total mass");
    total += link.mass;
  }
  return total;
}

void GravityFfModel::evaluate(const std::array<double, 4>& q_model, std::vector<double>& torque,
                              double& energy) const {
  torque.assign(movable_names_.size(), 0.0);
  energy = 0.0;
  std::map<std::string, Frame> frames;
  std::map<std::string, std::vector<ChainEntry>> chains;
  frames[root_] = Frame{identity(), {0, 0, 0}};
  chains[root_] = {};
  std::vector<bool> done(joints_.size(), false);
  std::size_t remaining = joints_.size();
  while (remaining > 0) {
    bool progressed = false;
    for (std::size_t position = 0; position < joints_.size(); ++position) {
      if (done[position]) continue;
      const GravityJoint& joint = joints_[position];
      const auto parent = frames.find(joint.parent);
      if (parent == frames.end()) continue;
      Frame world;
      world.rotation = multiply(parent->second.rotation, rotation_xyz(joint.rpy));
      world.position = add(parent->second.position, rotate(parent->second.rotation, joint.xyz));
      auto chain = chains[joint.parent];
      if (joint.revolute) {
        world.rotation = multiply(world.rotation, rotation_axis(joint.axis, joint.offset));
        const std::size_t index = movable_index(joint.name);
        const double angle = q_model[index];
        if (!(angle >= joint.lower && angle <= joint.upper))
          throw std::invalid_argument("Gravity model joint outside URDF limits: " + joint.name);
        chain.push_back(ChainEntry{index, world.position, rotate(world.rotation, joint.axis)});
        world.rotation = multiply(world.rotation, rotation_axis(joint.axis, angle));
      }
      frames[joint.child] = world;
      chains[joint.child] = chain;
      for (const auto& link : links_) {
        if (link.name != joint.child) continue;
        if (link.effective_parameters) {
          const auto moment = rotate(world.rotation, link.first_moment);
          Vec3 weighted_center{};
          for (std::size_t k = 0; k < 3; ++k)
            weighted_center[k] = link.mass * world.position[k] + moment[k];
          energy -= dot(gravity_, weighted_center);
          for (const auto& entry : chain) {
            Vec3 weighted_arm{};
            for (std::size_t k = 0; k < 3; ++k)
              weighted_arm[k] = weighted_center[k] - link.mass * entry.pivot[k];
            torque[entry.index] -= dot(entry.axis, cross(weighted_arm, gravity_));
          }
          continue;
        }
        const Vec3 center = add(world.position, rotate(world.rotation, link.com));
        const Vec3 force{link.mass * gravity_[0], link.mass * gravity_[1], link.mass * gravity_[2]};
        energy -= dot(force, center);
        for (const auto& entry : chain)
          torque[entry.index] -= dot(entry.axis, cross(sub(center, entry.pivot), force));
      }
      done[position] = true;
      --remaining;
      progressed = true;
    }
    if (!progressed) throw std::invalid_argument("Gravity model kinematic tree is disconnected");
  }
}

std::vector<double> GravityFfModel::torque(const std::array<double, 4>& q_model) const {
  std::vector<double> torque;
  double energy = 0;
  evaluate(q_model, torque, energy);
  return torque;
}

double GravityFfModel::potential_energy(const std::array<double, 4>& q_model) const {
  std::vector<double> torque;
  double energy = 0;
  evaluate(q_model, torque, energy);
  return energy;
}
}  // namespace slaihand_hardware
