#include "boids.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>

int size(std::vector<BoidState> const& v) {
  return (static_cast<int>(v.size()));
}

int size(std::vector<double> const& v) { return (static_cast<int>(v.size())); }

std::vector<BoidState> neighborscontrol(std::vector<BoidState> const& pesci,
                                        BoidState const& b1, double const d) {
  auto p = pesci;

  p.erase(std::remove_if(
              p.begin(), p.end(),
              [&b1, d](BoidState const& b) { return (norm(b1, b) > d); }),
          p.end());

  return p;
}

bool same_pos_check(BoidState const& b1, std::vector<BoidState> const& boids) {
  auto same_position_it = std::find_if(
      boids.begin(), boids.end(),
      [&b1](BoidState const& b) { return b.x == b1.x && b.y == b1.y; });
  return same_position_it != boids.end() ? false : true;
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
  return it != boid.end() ? false : true;
}

std::vector<BoidState> velocity_limit(std::vector<BoidState>& boidsvec) {
  std::transform(
      boidsvec.begin(), boidsvec.end(), boidsvec.begin(), [](BoidState& b) {
        if (b.v_x > 5.) {
          b.v_x = 5.;
        } else if (b.v_x < -5.) {
          b.v_x = -5.;
        } else if (b.v_x > -0.5 && b.v_x < 0.) {
          b.v_x = -0.5;
        } else if (b.v_x > 0. && b.v_x < 0.5) {
          b.v_x = 0.5;
        }

        if (b.v_y > 5.) {
          b.v_y = 5.;
        } else if (b.v_y < -5.) {
          b.v_y = -5.;
        } else if (b.v_y > -0.5 && b.v_y < 0.) {
          b.v_y = -0.5;
        } else if (b.v_y > 0. && b.v_y < 0.5) {
          b.v_y = 0.5;
        }

        assert(b.v_x <= 5. && b.v_x >= -5. && b.v_y <= 5. && b.v_y >= -5.);
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

// calculates the new coordinates of position and velocity of a boid using law
// of motion
BoidState Boids::singleboid(std::vector<BoidState> const& vec,
                            BoidState const& b1, double const delta_t) const {
  if (size(vec) > 1) {
    Components v_old = {b1.v_x, b1.v_y};
    auto v_1 = s_(vec, b1);
    auto v_2 = a_(vec, b1);
    auto v_3 = c_(vec, b1);
    auto v_new = v_old + v_1 + v_2 + v_3;
    return {b1.x + v_new.val_x * delta_t, b1.y + v_new.val_y * delta_t,
            v_new.val_x, v_new.val_y};
  } else {
    return {b1.x + b1.v_x * delta_t, b1.y + b1.v_y * delta_t, b1.v_x, b1.v_y};
  }
}

std::vector<BoidState> Boids::totalboids() const { return boids_; }

void Boids::pushback(BoidState const& boid) {
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
    auto nearfishes = neighborscontrol(boids_, fish, d_);
    fishes.push_back(singleboid(nearfishes, fish, delta_t));
  }

  borders(fishes);

  velocity_limit(fishes);

  assert(size(fishes) == size(boids_));
  boids_ = fishes;
}

void Boids::setvector(std::vector<BoidState> const& b) { boids_ = b; }

Stats statistic(Boids& b, double const delta_t) {
  b.evolution(delta_t);
  auto const& vec = b.totalboids();
  std::vector<double> distances{};
  assert(size(vec) != 0);
  auto it = vec.begin();
  for (; it != vec.end(); ++it) {
    auto it_2 = std::next(it);
    for (; it_2 != vec.end(); ++it_2) {
      distances.push_back(norm(*it, *it_2));
      ;
    }
  }

  assert(size(distances) != 0);

  auto mean_dist = (std::accumulate(distances.begin(), distances.end(), 0.)) /
                   size(distances);

  auto mean_dist2 = (std::inner_product(distances.begin(), distances.end(),
                                        distances.begin(), 0.)) /
                    size(distances);

  // mean standard deviation
  auto std_dist = std::sqrt(mean_dist2 - mean_dist * mean_dist) /
                  std::sqrt(size(distances));

  auto sum =
      std::accumulate(vec.begin(), vec.end(), BoidState{0.0, 0.0, 0.0, 0.0});
  Components mean_vel{sum.v_x / size(vec), sum.v_y / size(vec)};

  std::vector<double> velocities{};

  for (auto i : vec) {
    velocities.push_back(velocity_norm(i));
  }

  assert(size(velocities) != 0);
  auto mean_speed = std::sqrt(mean_vel.val_x * mean_vel.val_x +
                              mean_vel.val_y * mean_vel.val_y);

  auto mean_speed2 = (std::inner_product(velocities.begin(), velocities.end(),
                                         velocities.begin(), 0.)) /
                     size(velocities);
  auto std_speed = std::sqrt(mean_speed2 - mean_speed * mean_speed) /
                   std::sqrt(size(velocities));
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
