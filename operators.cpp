#include <cmath>

#include "boids.hpp"

BoidState& BoidState::operator+=(BoidState const& other) {
  x += other.x;
  y += other.y;
  v_x += other.v_x;
  v_y += other.v_y;
  return *this;
}

BoidState& BoidState::operator*=(double const other) {
  x *= other;
  y *= other;
  v_x *= other;
  v_y *= other;
  return *this;
}

BoidState& BoidState::operator*=(BoidState const& other) {
  x *= other.x;
  y *= other.y;
  v_x *= other.v_x;
  v_y *= other.v_y;
  return *this;
}

bool operator==(BoidState const& b1, BoidState const& b2) {
  return {b1.x == b2.x && b1.y == b2.y && b1.v_x == b2.v_x && b1.v_y == b2.v_y};
}

bool operator!=(BoidState const& b1, BoidState const& b2) {
  return {b1.x != b2.x || b1.y != b2.y || b1.v_x != b2.v_x || b1.v_y != b2.v_y};
}

BoidState operator+(BoidState const& b1, BoidState const& b2) {
  auto result = b1;
  return result += b2;
}

BoidState operator-(BoidState const& b1, BoidState const& b2) {
  return {b1.x - b2.x, b1.y - b2.y, b1.v_x - b2.v_x, b1.v_y - b2.v_y};
}

BoidState operator*(BoidState const& b1, double const d) {
  auto result = b1;
  return result *= d;
}

BoidState operator*(BoidState const& b1, BoidState const& b2) {
  auto result = b1;
  return result *= b2;
}

double norm(BoidState const& b1, BoidState const& b2) {
  auto result = (b1.x - b2.x) * (b1.x - b2.x) + (b1.y - b2.y) * (b1.y - b2.y);
  assert(!(result < 0));
  return std::sqrt(result);
}
// magnitude of velocity
double velocity_norm(BoidState const& b) {
  auto result = (b.v_x * b.v_x + b.v_y * b.v_y);
  return std::sqrt(result);
}

Components& Components::operator+=(Components const& other) {
  val_x += other.val_x;
  val_y += other.val_y;
  return *this;
}

Components& Components::operator*=(double const d) {
  val_x *= d;
  val_y *= d;
  return *this;
}

Components operator+(Components const& c1, Components const& c2) {
  auto result = c1;
  return result += c2;
}

Components operator-(Components const& c1, Components const& c2) {
  return {c1.val_x - c2.val_x, c2.val_y - c2.val_y};
}

Components operator*(Components const& c1, double const d) {
  auto result = c1;
  return result *= d;
}

bool operator==(Components const& c1, Components const& c2) {
  return {c1.val_x == c2.val_x && c1.val_y == c2.val_y};
}

bool operator!=(Components const& c1, Components const& c2) {
  return {c1.val_x != c2.val_x || c1.val_y != c2.val_y};
}
