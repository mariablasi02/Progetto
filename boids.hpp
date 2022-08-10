#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cassert>
#include <cmath>
#include <stdexcept>

// manca il controllo degli errori su tutto

struct BoidState {
  double x{};
  double y{};
  double v_x{};
  double v_y{};
  BoidState& operator+=(BoidState const& other) {
    x += other.x;
    y += other.y;
    v_x += other.v_x;
    v_y += other.v_y;
    return *this;
  }
};

BoidState operator==(BoidState const& b1, BoidState const& b2) {
  return {b1.x == b2.x, b1.y == b2.y, b1.v_x == b2.v_x, b1.v_y == b2.v_y};
}

BoidState operator!=(BoidState const& b1, BoidState const& b2) {
  return {b1.x != b2.x || b1.y != b2.y || b1.v_x != b2.v_x ||
          b1.v_y != b2.v_y};  // così basta che solo una delle quattro
                              // condizioni non sia vera
}

BoidState operator+(BoidState const& b1, BoidState const& b2) {
  auto result = b1;
  return result += b2;
}

BoidState operator-(BoidState const& b1, BoidState const& b2) {
  return {b1.x - b2.x, b1.y - b2.y, b1.v_x - b2.v_x, b1.v_y - b2.v_y};
}
BoidState operator*(BoidState const& b1, BoidState const& b2) {
  return {b1.x * b2.x, b1.y * b2.y, b1.v_x * b2.v_x, b1.v_y * b2.v_y};
}
BoidState operator/(BoidState const& b1, BoidState const& b2) {
  if (b2.x == 0 || b2.y == 0 || b2.v_x == 0 || b2.v_y == 0) {
    throw std::runtime_error{"Denominator is zero"};
  }
  return {b1.x / b2.x, b1.y / b2.y, b1.v_x / b2.v_x, b1.v_y / b2.v_y};
}

double norm(BoidState const& b1, BoidState const& b2) {
  auto result =
      std::abs((b1.x - b2.x) * (b1.x - b2.x) + (b1.y - b2.y) * (b1.y - b2.y));
  assert(result > 0);  // non so se abbia senso ora mantenere questo assert
                       // visto che ho messo ||
  return std::sqrt(result);
}

// nelle classi private si sceglie di mantenere _ alla fine dei data members

struct VelocityComponents {
  double vel_x;
  double vel_y;
};

class SeparationRule {
  int const n_;
  double const separation_const_;

 public:
  double const distance_s{};  // valutare un valore che viene deciso da noi->
                              // valutare se mettere pubblico o privato

  SeparationRule(int const n, double const s) : n_{n}, separation_const_{s} {}

  VelocityComponents operator()(BoidState const& b1,
                                BoidState const& b2) const {
    return;
  }
};

class AllignmentRule {};

class CohesionRule {};

class Boids {};

#endif
