#include "boids.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <string>

int size(std::vector<BoidState> const& v) {
  return (static_cast<int>(v.size()));
}

std::vector<BoidState> NeighborsControl(std::vector<BoidState> const& pesci,
                                        BoidState const& b1, double const d) {
  auto p = pesci;
  auto n = size(pesci);

  p.erase(std::remove_if(p.begin(), p.end(),
                         [&b1, d](BoidState const& b) {
                           return (norm(b1, b) > d);
                         }),  // provato a sistemare by ref by value, funzia?
          p.end());
  assert(size(pesci) == n);

  return p;
}

bool same_pos_check(BoidState const& b1, std::vector<BoidState> const& boids) {
  auto same_position_it =
      std::find_if(boids.begin(), boids.end(), [&b1](BoidState const& b) {
        return b.x == b1.x && b.y == b1.y;
      });  // uguale a sopra per le lambda
  if (same_position_it != boids.end()) {
    return false;
  } else {
    return true;
  }
}

bool same_pos_check(std::vector<BoidState> const& boid) {
  auto it = boid.begin();
  for (; it != boid.end(); ++it) {
    auto c = *it;
    auto it_ = std::next(it);
    auto same_pos_it = std::find_if(it_, boid.end(), [&c](BoidState const& n) {
      return c.x == n.x && c.y == n.y;
    });
    if (same_pos_it != boid.end()) {
      break;
    }
  }
  if (it != boid.end()) {
    return false;
  } else {
    return true;
  }
}

std::vector<BoidState> velocity_limit(std::vector<BoidState>& boidsvec) {
  std::transform(boidsvec.begin(), boidsvec.end(), boidsvec.begin(),
                 [](BoidState& b) {
                   if (b.v_x < -5.) {
                     b.v_x = -5;
                   } else if (b.v_x > 5.) {
                     b.v_x = 5;
                   } else if (b.v_x > -0.5 || b.v_x < 0.5) {
                     b.v_x = 0.5;
                   }
                   if (b.v_y < -5.) {
                     b.v_y = -5;
                   } else if (b.v_y > 5) {
                     b.v_y = 5;
                   } else if (b.v_y > -0.5 || b.v_y < 0.5) {
                     b.v_y = 0.5;
                   }
                   return BoidState{b.x, b.y, b.v_x, b.v_y};
                 });
  return boidsvec;
}

std::vector<BoidState> borders(std::vector<BoidState>& v) {
  std::transform(v.begin(), v.end(), v.begin(), [](BoidState& b) {
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

BoidState Boids::singleboid(std::vector<BoidState> const& vec,
                            BoidState const& b1, double const delta_t) const {
  if (size(vec) > 1) {
    Components v_old = {b1.v_x, b1.v_y};
    auto v_1 = s_(vec, b1);
    auto v_2 = a_(vec, b1);
    auto v_3 = c_(vec, b1);
    auto v_new = v_old + v_1 + v_2 + v_3;
    if (std::abs(v_new.val_x) > 10.) {
      auto x = 2 / std::abs(v_new.val_x);
      v_new.val_x *= x;
    }
    if (std::abs(v_new.val_y) > 10.) {
      auto y = 2 / std::abs(v_new.val_y);
      v_new.val_y *= y;
    }
    return {b1.x + v_new.val_x * delta_t, b1.y + v_new.val_y * delta_t,
            v_new.val_x, v_new.val_y};
  } else {
    return {b1.x + b1.v_x * delta_t, b1.y + b1.v_y * delta_t, b1.v_x, b1.v_y};
  }
}

std::vector<BoidState> Boids::TotalBoids() const { return boids_; }

void Boids::push_back(BoidState const& boid) {
  if (same_pos_check(boid, boids_) == true) {
    boids_.push_back(boid);
  } else {
    throw std::runtime_error{"Error: this boid is already in the goup"};
  }
}

void Boids::evolution(double const delta_t) {
  if (delta_t < 0 || delta_t == 0) {
    throw std::runtime_error{"Time must be a positive value"};
  }
  std::vector<BoidState> fishes;
  for (auto fish : boids_) {
    auto nearfishes = NeighborsControl(boids_, fish, d_);
    fishes.push_back(singleboid(nearfishes, fish, delta_t));
  }

  borders(fishes);

  velocity_limit(fishes);

  assert(size(fishes) == size(boids_));
  boids_ = fishes;
  assert(same_pos_check(boids_));
}

void Boids::setvector(std::vector<BoidState> const& b) {  // prova
  boids_ = b;
}

Stats statistic(Boids& b, double const delta_t) {
  b.evolution(delta_t);
  auto vec = b.TotalBoids();
  std::vector<double> distances{};
  assert(vec.size() != 0);
  auto it = vec.begin();
  for (; it != vec.end(); ++it) {
    auto it_2 = std::next(it);
    for (; it_2 != vec.end(); ++it_2) {
      distances.push_back(norm(*it, *it_2));
      ;
    }
  }
  assert(distances.size() != 0);
  auto mean_dist = (std::accumulate(distances.begin(), distances.end(), 0.)) /
                   static_cast<int>(distances.size());

  auto mean_dist2 = (std::inner_product(distances.begin(), distances.end(),
                                        distances.begin(), 0.)) /
                    static_cast<int>(distances.size());
  auto std_dist = std::sqrt(mean_dist2 - mean_dist * mean_dist) /
                  std::sqrt(static_cast<int>(distances.size()));

  auto sum =
      std::accumulate(vec.begin(), vec.end(), BoidState{0.0, 0.0, 0.0, 0.0});
  Components mean_vel{sum.v_x / size(vec), sum.v_y / size(vec)};

  std::vector<double> velocities{};

  for (auto i : vec) {
    velocities.push_back(velocity_norm(i));
  }

  assert(velocities.size() != 0);
  auto mean_speed = std::sqrt(mean_vel.val_x * mean_vel.val_x +
                              mean_vel.val_y * mean_vel.val_y);

  auto mean_speed2 = (std::inner_product(velocities.begin(), velocities.end(),
                                         velocities.begin(), 0.)) /
                     static_cast<int>(velocities.size());
  auto std_speed = std::sqrt(mean_speed2 - mean_speed * mean_speed) /
                   std::sqrt(static_cast<int>(velocities.size()));
  Stats data{mean_dist, std_dist, mean_speed, std_speed};
  return data;
}

std::string state(Boids& b, double const delta_t) {
  auto data = statistic(b, delta_t);

  return "Mean distance and standard deviation: " +
         std::to_string(data.mean_distance) + " +/- " +
         std::to_string(data.std_distance) + '\n' +
         "Mean speed and standard deviation: " +
         std::to_string(data.mean_speed) + " +/- " +
         std::to_string(data.std_speed) + '\n';
}

std::vector<std::string> simulate(Boids& b, double duration, int steps,
                                  int prescale) {
  std::vector<std::string> b_states;
  double delta_t{duration / steps};
  for (int step = 0; step != steps; ++step) {
    if (step % prescale == 0) {
      b_states.push_back(state(b, delta_t));  // state of the chain after
                                              // delta_t
    } else {
      b.evolution(delta_t);
    }
  }
  return b_states;
}
