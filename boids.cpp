#include "boids.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>


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

BoidState& BoidState::operator*=(BoidState const other) {
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
  return {c1.val_x != c2.val_x && c1.val_y != c2.val_y};
}

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
                                        BoidState b1, double d) {
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

AllignmentRule Boids::a() const {
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

void Boids::evolution(double const delta_t) {
  Boids b{n(), d(), s(), a(), c()};
  std::vector<BoidState> fishes;
  for (auto fish : boids_) {
    auto nearfishes = NeighborsControl(boids_, fish, d_);
    fishes.push_back(b.singleboid(nearfishes, fish, delta_t));
  }
  assert(size(fishes) == size(boids_));
  boids_ = fishes;
}

void Boids::setvector(std::vector<BoidState> const& b) {  // prova
  boids_ = b;
}

void state(Boids& b, double const delta_t) {
  b.evolution(delta_t);
  auto vec = b.TotalBoids();
  auto sum = std::accumulate(
      vec.begin(), vec.end(),
      BoidState{0.0, 0.0, 0.0,
                0.0});  // forse da ripensare come distanza tra boids
  Components mean_pos{sum.x / size(vec), sum.y / size(vec)};
  Components mean_vel{sum.v_x / size(vec), sum.v_y / size(vec)};

  auto mean_position =
      std::sqrt(mean_pos.val_x * mean_pos.val_x +  // aspettare confronto
                mean_pos.val_y * mean_pos.val_y);
  auto mean_velocity = std::sqrt(mean_vel.val_x * mean_vel.val_x +
                                 mean_vel.val_y * mean_vel.val_y);
  auto products =
      std::inner_product(vec.begin(), vec.end(), vec.begin(),
                         BoidState{0.0, 0.0, 0.0, 0.0});  // somma dei quadrati
  auto variance = products * (1 / size(vec)) -
                  sum * (1 / size(vec)) * sum * (1 / size(vec));

  assert((mean_pos.val_x * mean_pos.val_x + mean_pos.val_y * mean_pos.val_y) !=
             0 &&
         (mean_vel.val_x * mean_vel.val_x + mean_vel.val_y * mean_vel.val_y) !=
             0);

  auto std_dev_position = std::sqrt(
      (mean_pos.val_x * mean_pos.val_x) /
      (mean_pos.val_x * mean_pos.val_x + mean_pos.val_y * mean_pos.val_y) *
      (variance.x * variance.x + variance.y * variance.y) / size(vec));
  auto std_dev_velocity = std::sqrt(
      (mean_vel.val_x * mean_vel.val_x) /
      (mean_vel.val_x * mean_vel.val_x + mean_vel.val_y * mean_vel.val_y) *
      (variance.x * variance.x + variance.y * variance.y) / size(vec));


 /* std::vector<double> sums;
 auto it = vec.begin();
 auto it2 = vec.begin();
    for (; it2 != vec.end(); ++it2){
 for (auto i : vec){
        sums.push_back(norm(i, *it2));
    }
    //++it;
 } */


  /* double somma_x;
  double somma_y;
  double somma_v_x;
  double somma_v_y;
  auto it=vec.begin();
  for (; it != vec.end(); ++it) {
    somma_x += (vec.begin()->x - mean_pos.val_x);
    somma_y += (vec.begin()->y - mean_pos.val_y);
    somma_v_x += (vec.begin()->v_x - mean_vel.val_x);
    somma_v_y += (vec.begin()->v_y - mean_vel.val_y);
  }
  auto std_dev_x = std::sqrt(somma_x * somma_x / size(vec));
  auto std_dev_y = std::sqrt(somma_y * somma_y / size(vec));
  auto std_dev_position = //non si dovrebbe propagare in quadratura?
      std::sqrt(std_dev_x * std_dev_x + std_dev_y * std_dev_y);
  auto std_dev_v_x = std::sqrt(somma_v_x * somma_v_x / size(vec));
  auto std_dev_v_y = std::sqrt(somma_v_y * somma_v_y / size(vec));
  auto std_dev_velocity =
      std::sqrt(std_dev_v_x * std_dev_v_x + std_dev_v_y * std_dev_v_y); */

  std::cout << '\n'
            << "Mean position and standard deviation: " << mean_position
            << " +/- " << std_dev_position << '\n';
  std::cout << '\n'
            << "Mean velocity and stardand deviation: " << mean_velocity
            << " +/- " << std_dev_velocity << '\n';
}
