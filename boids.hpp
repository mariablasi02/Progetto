#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>
#include <typeinfo>

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

  BoidState& operator*=(double const other) {
    x *= other;
    y *= other;
    v_x *= other;
    v_y *= other;
    return *this;
  }

  BoidState& operator*=(BoidState const other) {
    x *= other.x;
    y *= other.y;
    v_x *= other.v_x;
    v_y *= other.v_y;
    return *this;
  }
};

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

// nelle classi private si sceglie di mantenere _ alla fine dei data members

struct Components {
  double val_x;
  double val_y;

  Components& operator+=(Components const& other) {
    val_x += other.val_x;
    val_y += other.val_y;
    return *this;
  }

  Components& operator*=(double const d) {
    val_x *= d;
    val_y *= d;
    return *this;
  }
};

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

// idea : si potrebbero implementare le classi delle regole già con dei vettori
// che restituiscono le regole -> dopo mando audio

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
  //}
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

class SeparationRule {
  double const s_;
  double const distance_s_;
  // valutare un valore che viene deciso da noi

 public:
  SeparationRule(double const s, double d_s) : s_{s}, distance_s_{d_s} {}
  auto operator()(std::vector<BoidState> const& b, BoidState const& b1) const {
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
};

bool check_ownership(std::vector<BoidState> const& cont, BoidState const& c) {
  if (!cont.empty()) {
    auto it = std::find(cont.begin(), cont.end(), c);
    return it != cont.end();
  } else {
    return false;
  }
}

class AllignmentRule {
  double const a_;

 public:
  AllignmentRule(double const a) : a_{a} {
    if (a == 1. || a > 1.) {
      throw std::runtime_error{"a must be < than 1"};
    }
  };

  auto get_a() const { return a_; }
  Components operator()(std::vector<BoidState> boids,
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
};

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

class CohesionRule {
  double const cohesion_const_;

 public:
  CohesionRule(double const c) : cohesion_const_{c} {}

  Components operator()(std::vector<BoidState> const& cboids,
                        BoidState const& b1) const {
    assert(check_ownership(cboids, b1));

    Components position_of_c = COM(cboids, b1);

    BoidState com{position_of_c.val_x, position_of_c.val_y, 0., 0.};
    BoidState result = (com - b1) * cohesion_const_;
    return {result.x, result.y};
  }
};

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

class Boids {
  int const n_;
  double const d_;
  SeparationRule s_;
  AllignmentRule a_;
  CohesionRule c_;
  std::vector<BoidState> boids_;

 public:
  Boids(int const n, double const d, SeparationRule const& s,
        AllignmentRule const& a, CohesionRule const& c)
      : n_{n}, d_{d}, s_{s}, a_{a}, c_{c} {
    assert(n > 1);
    assert(a_.get_a() < 1.);
  }

  BoidState singleboid(std::vector<BoidState> const& vec, BoidState const& b1,
                       double const delta_t) const {
    if (size(vec) > 1) {
      Components v_old = {b1.v_x, b1.v_y};
      auto v_1 = s_(vec, b1);
      auto v_2 = a_(vec, b1);
      auto v_3 = c_(vec, b1);
      auto v_new = v_old + v_1 + v_2 + v_3;
      //assert(v_1.val_x != 0. && v_1.val_y != 0.);
      //assert(v_2.val_x != 0. && v_2.val_y != 0.);
      //assert(v_3.val_x != 0. && v_3.val_y != 0.);
      return {b1.x + v_new.val_x * delta_t, b1.y + v_new.val_y * delta_t,
              v_new.val_x, v_new.val_y};
    } else {
      return {b1.x + b1.v_x * delta_t, b1.y + b1.v_y * delta_t, b1.v_x, b1.v_y};
    }
  }

  std::vector<BoidState> TotalBoids() const { return boids_; }
  int n() const { return n_; }
  double d() const { return d_; }
  SeparationRule s() const { return s_; }
  AllignmentRule a() const {
    assert(a_.get_a() < 1.);
    return a_;
  }
  CohesionRule c() const { return c_; }

  void push_back(BoidState const& boid) {
    if (same_pos_check(boid, boids_) == true) {
      boids_.push_back(boid);
    } else {
      throw std::runtime_error{"Error: this boid is already in the goup"};
    }
  }

  void evolution(double const delta_t) {
    Boids b{n(), d(), s(), a(), c()};
    std::vector<BoidState> fishes;
    for (auto fish : boids_) {
      auto nearfishes = NeighborsControl(boids_, fish, d_);
      fishes.push_back(b.singleboid(nearfishes, fish, delta_t));
    }
    assert(size(fishes) == size(boids_));
    boids_ = fishes;
  }
};

void state(Boids& b, double const delta_t) {
  b.evolution(delta_t);
  auto vec = b.TotalBoids();
  auto sum = std::accumulate(
      vec.begin(), vec.end(),
      BoidState{0.0, 0.0, 0.0,
                0.0});  // forse da ripensare come distanza tra boids
  Components mean_pos{sum.x / size(vec), sum.y / size(vec)};
  Components mean_vel{sum.v_x / size(vec), sum.v_y / size(vec)};

  auto mean_position = std::sqrt(mean_pos.val_x * mean_pos.val_x + //aspettare confronto
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

  std::cout << "Mean position and standard deviation: " << mean_position
            << " +/- " << std_dev_position;
  std::cout << "Mean velocity and stardand deviation: " << mean_velocity
            << " +/- " << std_dev_velocity;
}

#endif
