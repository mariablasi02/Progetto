#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cmath>

// manca il controllo degli errori su tutto

struct BoidState {
  double x{};
  double y{};
  double v_x{};
  double v_y{};
};

BoidState operator+(BoidState const &b1, BoidState const &b2) {
  return {b1.x + b2.x, b1.y + b2.y, b1.v_x + b2.v_x, b1.v_y + b2.v_y};
}

BoidState operator-(BoidState const &b1, BoidState const &b2) {
  return {b1.x - b2.x, b1.y - b2.y, b1.v_x - b2.v_x, b1.v_y - b2.v_y};
}
BoidState operator*(BoidState const &b1, BoidState const &b2) {
  return {b1.x * b2.x, b1.y * b2.y, b1.v_x * b2.v_x, b1.v_y * b2.v_y};
}
BoidState operator/(BoidState const &b1, BoidState const &b2) {
  return {b1.x / b2.x, b1.y / b2.y, b1.v_x / b2.v_x, b1.v_y / b2.v_y};
}

double norm(BoidState const &b1, BoidState const &b2) {
  auto result = (b1.x - b2.x) * (b1.x - b2.x) + (b1.y - b2.y) * (b1.y - b2.y);
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

  VelocityComponents operator()(BoidState const &b1,
                                BoidState const &b2) const {
    return;
  }
};

class AllignmentRule {};

class CohesionRule {};

class Boids {};

#endif
