#include "boids.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>


Components SeparationRule::operator()(std::vector<BoidState> const& b,
                                      BoidState const& b1) const {
  assert(size(b) > 1);
  auto boids = NeighborsControl(b, b1, distance_s_);
  auto boid_it = boids.begin();
  auto boid_it_last = boids.end();

  std::vector<double> boidsdiff_x;
  std::vector<double> boidsdiff_y;

  for (; boid_it != boid_it_last; ++boid_it) {
    double diff_x = (b1.x - boid_it->x);
    double diff_y = (b1.y - boid_it->y);

    boidsdiff_x.push_back(diff_x);
    boidsdiff_y.push_back(diff_y);
  }

  double sum_x = std::accumulate(boidsdiff_x.begin(), boidsdiff_x.end(), 0.);
  double sum_y = std::accumulate(boidsdiff_y.begin(), boidsdiff_y.end(), 0.);

  return Components{-s_ * sum_x, -s_ * sum_y};
}

bool check_ownership(std::vector<BoidState> const& cont, BoidState const& c) {
  if (!cont.empty()) {
    auto it = std::find(cont.begin(), cont.end(), c);
    return it != cont.end();
  } else {
    return false;
  }
}

double AlignmentRule::get_a() const { return a_; }

Components AlignmentRule::operator()(std::vector<BoidState> boids,
                                      BoidState const& b1) const {
  assert(check_ownership(boids, b1));
  if (size(boids) > 1) {
    BoidState sum = std::accumulate(boids.begin(), boids.end(),
                                    BoidState{0., 0., 0., 0.} - b1);
    return Components{((sum.v_x / (size(boids) - 1)) - b1.v_x) * a_,
                      ((sum.v_y / (size(boids) - 1)) - b1.v_y) * a_};
  } else {
    return Components{0., 0.};
  }
}

Components COM(std::vector<BoidState> const& vec, BoidState const& b1) {
  BoidState sum =
      std::accumulate(vec.begin(), vec.end(), BoidState{0., 0., 0., 0.}) - b1;
  if ((size(vec)) > 1) {

    double den = (static_cast<double>(size(vec)) - 1.);

    return {sum.x / den, sum.y / den};
  } else {
    return {b1.x, b1.y};
  }
}

Components CohesionRule::operator()(std::vector<BoidState> const& cboids,
                                    BoidState const& b1) const {
  assert(check_ownership(cboids, b1));

  Components position_of_c = COM(cboids, b1);

  BoidState com{position_of_c.val_x, position_of_c.val_y, 0., 0.};
  BoidState result = (com - b1) * cohesion_const_;
  return {result.x, result.y};
}
