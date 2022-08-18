#include <algorithm>
#include <cmath>
#include <numeric>

#include "boids.hpp"

bool check_ownership(std::vector<BoidState> const& cont, BoidState const& c) {
  if (!cont.empty()) {
    auto it = std::find(cont.begin(), cont.end(), c);
    return it != cont.end();
  } else {
    return false;
  }
}

Components SeparationRule::operator()(std::vector<BoidState> const& b,
                                      BoidState const& b1) const {
  assert(size(b) > 1);
  assert(check_ownership(b, b1));
  auto boids = NeighborsControl(b, b1, distance_s_);
  auto boid_it = boids.begin();

  std::vector<double> boidsdiff_x;
  std::vector<double> boidsdiff_y;

  for (; boid_it != boids.end(); ++boid_it) {
    auto diff_x = (b1.x - boid_it->x);
    auto diff_y = (b1.y - boid_it->y);

    boidsdiff_x.push_back(diff_x);
    boidsdiff_y.push_back(diff_y);
  }

  auto sum_x = std::accumulate(boidsdiff_x.begin(), boidsdiff_x.end(), 0.);
  auto sum_y = std::accumulate(boidsdiff_y.begin(), boidsdiff_y.end(), 0.);

  return Components{-s_ * sum_x, -s_ * sum_y};
}

double AlignmentRule::get_a() const { return a_; }

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
  auto den = (static_cast<double>(size(vec)) - 1.);
  auto sum =
      std::accumulate(vec.begin(), vec.end(), BoidState{0., 0., 0., 0.}) - b1;

  return {sum.x / den, sum.y / den};
}

Components CohesionRule::operator()(std::vector<BoidState> const& cboids,
                                    BoidState const& b1) const {
  assert(size(cboids) > 1);
  assert(check_ownership(cboids, b1));

  auto position_of_c = centre_of_mass(cboids, b1);

  BoidState com{position_of_c.val_x, position_of_c.val_y, 0., 0.};
  auto result = (com - b1) * cohesion_const_;
  return {result.x, result.y};
}
