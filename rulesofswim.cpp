#include <algorithm>
#include <cmath>
#include <numeric>

#include "boids.hpp"

// check if c belongs to cont
bool check_ownership(std::vector<BoidState> const& cont, BoidState const& c) {
  if (!cont.empty()) {
    auto it = std::find(cont.begin(), cont.end(), c);
    return it != cont.end();
  } else {
    return false;
  }
}
// calculates v_1
Components SeparationRule::operator()(std::vector<BoidState> const& b,
                                      BoidState const& b1) const {
  assert(size(b) > 1);
  assert(check_ownership(b, b1));

  auto vec = neighborscontrol(b, b1, distance_s_);
  std::transform(vec.begin(), vec.end(), vec.begin(),
                 [&b1](BoidState& j) { return (j - b1); });

  auto sum = std::accumulate(vec.begin(), vec.end(), BoidState{0., 0., 0., 0.});
  return {-s_ * sum.x, -s_ * sum.y};
}

double AlignmentRule::get_a() const { return a_; }
// calculates v_2
Components AlignmentRule::operator()(std::vector<BoidState> const& boids,
                                     BoidState const& b1) const {
  assert(size(boids) > 1);
  assert(check_ownership(boids, b1));
  auto sum = std::accumulate(boids.begin(), boids.end(),
                             BoidState{0., 0., 0., 0.} - b1);
  return Components{((sum.v_x / (size(boids) - 1)) - b1.v_x) * a_,
                    ((sum.v_y / (size(boids) - 1)) - b1.v_y) * a_};
}

Components centre_of_mass(std::vector<BoidState> const& vec,
                          BoidState const& b1) {
  assert((size(vec)) > 1);
  auto den = ((size(vec)) - 1.);
  auto sum =
      std::accumulate(vec.begin(), vec.end(), BoidState{0., 0., 0., 0.}) - b1;

  return {sum.x / den, sum.y / den};
}
// calculates v_3
Components CohesionRule::operator()(std::vector<BoidState> const& cboids,
                                    BoidState const& b1) const {
  assert(size(cboids) > 1);
  assert(check_ownership(cboids, b1));

  auto position_of_c = centre_of_mass(cboids, b1);

  BoidState com{position_of_c.val_x, position_of_c.val_y, 0., 0.};
  auto result = (com - b1) * cohesion_const_;
  return {result.x, result.y};
}
