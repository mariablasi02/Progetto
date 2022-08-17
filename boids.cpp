#include "boids.hpp"

#include <string.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>

int size(std::vector<BoidState> const& v) {
  /* if (v.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::overflow_error {
      "size_t value cannot be stored in a variable of
          type int."};
    }
    else */
  // if (static_cast<int>(v.size()) > 1) {
  return (static_cast<int>(v.size()));  // risolvere problema eccezione
  //} else {
  //  throw std::runtime_error{"Error: n must be > 1"};
  // }
}

std::vector<BoidState> NeighborsControl(std::vector<BoidState> const& pesci,
                                        BoidState const& b1, double d) {
  auto p = pesci;
  auto n = pesci.size();
  // auto b1 = *p.begin();
  p.erase(std::remove_if(p.begin(), p.end(),
                         [b1, d](BoidState b) { return (norm(b1, b) > d); }),
          p.end());
  assert(pesci.size() == n);

  return p;
}

bool same_pos_check(BoidState const& b1, std::vector<BoidState> boids) {
  auto same_position_it =
      std::find_if(boids.begin(), boids.end(),
                   [b1](BoidState b) { return b.x == b1.x && b.y == b1.y; });
  if (same_position_it != boids.end()) {
    return false;
  } else {
    return true;
  }
}

void same_position(BoidState const& b1, std::vector<BoidState> boids) {
  auto it = boids.begin();
  for (; it != boids.end(); ++it) {
    auto same_position_it =
        std::find_if(boids.begin(), boids.end(),
                     [b1](BoidState b) { return b.x == b1.x && b.y == b1.y; });
    if (same_position_it != boids.end()) {
      boids.erase(same_position_it);
    }
    assert(same_pos_check(b1, boids));
  }
}

BoidState Boids::singleboid(std::vector<BoidState> const& vec,
                            BoidState const& b1, double const delta_t) const {
  if (size(vec) > 1) {
    Components v_old = {b1.v_x, b1.v_y};
    auto v_1 = s_(vec, b1);
    auto v_2 = a_(vec, b1);
    auto v_3 = c_(vec, b1);
    auto v_new = v_old + v_1 + v_2 + v_3;
    // assert(v_1.val_x != 0. && v_1.val_y != 0.);
    // assert(v_2.val_x != 0. && v_2.val_y != 0.);
    // assert(v_3.val_x != 0. && v_3.val_y != 0.);
    return {b1.x + v_new.val_x * delta_t, b1.y + v_new.val_y * delta_t,
            v_new.val_x, v_new.val_y};
  } else {
    return {b1.x + b1.v_x * delta_t, b1.y + b1.v_y * delta_t, b1.v_x, b1.v_y};
  }
}

std::vector<BoidState> Boids::TotalBoids() const { return boids_; }

int Boids::n() const { return n_; }

double Boids::d() const { return d_; }

SeparationRule Boids::s() const { return s_; }

AlignmentRule Boids::a() const {
  assert(a_.get_a() < 1.);
  return a_;
}

CohesionRule Boids::c() const { return c_; }

void Boids::push_back(BoidState const& boid) {
  if (same_pos_check(boid, boids_) == true) {
    boids_.push_back(boid);
  } else {
    throw std::runtime_error{"Error: this boid is already in the goup"};
  }
}

std::vector<BoidState> velocity_limit_(std::vector<BoidState>& b) {
  std::transform(b.begin(), b.end(), b.begin(), [](BoidState b_) {
    /* if(b_.v_x < 250 && b_.v_x > 0){
      b_.v_x = 250;
    } */
    if (b_.v_x > 100) {
      b_.v_x = 100;
    }
    /* if(b_.v_y < 250 && b_.v_y > 0){
      b_.v_y = 250;
    } */
    if (b_.v_y > 100) {
      b_.v_y = 100;
    }
    /* if(b_.v_x > -250 && b_.v_x < 0){
      b_.v_x = -250;
    } */
    if (b_.v_x < -100) {
      b_.v_x = -100;
    }
    /* if(b_.v_y > -250 && b_.v_y < 0){
      b_.v_y = -250;
    } */
    if (b_.v_y < -100) {
      b_.v_y = -100;
    }
    return BoidState{b_.x, b_.y, b_.v_x, b_.v_y};
  });
  return b;
}

std::vector<BoidState> borders(std::vector<BoidState>& v) {
  std::transform(v.begin(), v.end(), v.begin(), [](BoidState b) {
    if (b.x <= 0.) {
      b.x = 1179.;
    } else if (b.x >= 1179.) {
      b.x = 0.;
    }
    if (b.y <= 0.) {
      b.y = 691.;
    } else if (b.y >= 691.) {
      b.y = 0.;
    }
    assert(b.x >= 0. && b.x <= 1179. && b.y >= 0. && b.y <= 691.);
    return BoidState{b.x, b.y, b.v_x, b.v_y};
  });
  return v;
}

void Boids::evolution(double const delta_t) {
  Boids b{n(), d(), s(), a(), c()};
  std::vector<BoidState> fishes;
  for (auto fish : boids_) {
    auto nearfishes = NeighborsControl(boids_, fish, d_);
    fishes.push_back(b.singleboid(nearfishes, fish, delta_t));
  }

  borders(fishes);
  velocity_limit_(fishes);
  assert(size(fishes) == size(boids_));
  boids_ = fishes;
}

void Boids::setvector(std::vector<BoidState> const& b) {  // prova
  boids_ = b;
}

std::string state(Boids& b, double const delta_t) {
  b.evolution(delta_t);
  auto vec = b.TotalBoids();

  std::vector<double> position;
  std::vector<double> velocity;

  auto it = vec.begin();

  for (; it != vec.end(); ++it) {
    auto it_2 = std::next(it);
    for (; it_2 != vec.end(); ++it_2) {
      position.push_back(norm(*it, *it_2));
      ;
    }
  }
  auto mean_position = (std::accumulate(position.begin(), position.end(), 0.)) /
                       static_cast<int>(position.size());

  auto sums_pos2_medio = (std::inner_product(position.begin(), position.end(),
                                             position.begin(), 0.)) /
                         static_cast<int>(position.size());
  auto std_dev_position =
      std::sqrt(sums_pos2_medio - mean_position * mean_position);

  auto sum =
      std::accumulate(vec.begin(), vec.end(), BoidState{0.0, 0.0, 0.0, 0.0});
  Components mean_vel{sum.v_x / size(vec), sum.v_y / size(vec)};

  for (auto i : vec) {
    velocity.push_back(velocity_norm(i));
  }
  auto mean_velocity = std::sqrt(mean_vel.val_x * mean_vel.val_x +
                                 mean_vel.val_y * mean_vel.val_y);

  auto sums_vel2_medio = (std::inner_product(velocity.begin(), velocity.end(),
                                             velocity.begin(), 0.)) /
                         static_cast<int>(velocity.size());
  auto std_dev_velocity =
      std::sqrt(sums_vel2_medio - mean_velocity * mean_velocity);
  auto pos = std::to_string(mean_position);
  auto pos_stdev = std::to_string(std_dev_position);
  auto speed = std::to_string(mean_velocity);
  auto speed_stdev = std::to_string(std_dev_velocity);
  return "Mean position and standard deviation: " + pos + "+/-" + pos_stdev +
         '\n' + "Mean velocity and standard deviation: " + speed + "+/-" +
         speed_stdev + '\n';
}
