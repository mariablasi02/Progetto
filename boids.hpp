#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

// manca il controllo degli errori su tutto

struct BoidState {
  double x{};
  double y{};
  double v_x{};
  double v_y{};

  // BoidState(double x, double y, double vx, double vy) : x{x}, y{y}, v_x{vx},
  // v_y{vy} {};

  BoidState(double x, double y, double vx, double vy)
      : x{x}, y{y}, v_x{vx}, v_y{vy} {};

  BoidState& operator+=(BoidState const& other) {
    x += other.x;
    y += other.y;
    v_x += other.v_x;
    v_y += other.v_y;
    return *this;
  }

  double norm(BoidState const& other) {
    auto result = (x - other.x) * (x - other.x) + (y - other.y) * (y - other.y);
    assert(!(result < 0));
    return std::sqrt(result);
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
  return {b1.x * d, b1.y * d, b1.v_x * d, b1.v_y * d};
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
  auto result = (b1.x - b2.x) * (b1.x - b2.x) + (b1.y - b2.y) * (b1.y - b2.y);
  assert(!(result < 0));
  return std::sqrt(result);
}

// nelle classi private si sceglie di mantenere _ alla fine dei data members

struct VelocityComponents {
  double vel_x;
  double vel_y;
};

VelocityComponents operator+(VelocityComponents const& c1,
                             VelocityComponents const& c2) {
  return {c1.vel_x + c2.vel_x, c2.vel_y + c2.vel_y};
}

VelocityComponents operator-(VelocityComponents const& c1,
                             VelocityComponents const& c2) {
  return {c1.vel_x - c2.vel_x, c2.vel_y - c2.vel_y};
}

VelocityComponents operator*(VelocityComponents const& c1, double const d) {
  return {c1.vel_x * d, c1.vel_y * d};
}

bool operator==(VelocityComponents const& c1, VelocityComponents const& c2) {
  return {c1.vel_x == c2.vel_x && c1.vel_y == c2.vel_y};
}

// idea : si potrebbero implementare le classi delle regole già con dei vettori
// che restituiscono le regole -> dopo mando audio

class SeparationRule {
  int const n_;
  double const s_;
  double const distance_s_;
  // valutare un valore che viene deciso da noi

 public:
  SeparationRule(int const n, double const s, double d_s)
      : n_{n}, s_{s}, distance_s_{d_s} {
    if (n_ <= 1) {
      throw std::runtime_error{"Number of boids must be >1"};
    }
  }
  auto operator()(std::vector<BoidState> boids, BoidState const& b1) const {
    auto boid_it = boids.begin();
    // auto boid_it_next = std::next(boids.begin());
    // auto boid_it_last = std::prev(boids.end());
    auto boid_it_last = boids.end();

    /* for (; boid_it_next != boid_it_last; ++boid_it_next) {
       double diff = norm(*boid_it, *boid_it_next);
       if (diff < distance_s_) {
         std::vector<BoidState> boids;

        std::vector<double> boidsdiff_x;
        std::vector<double> boidsdiff_y;

         for (; boid_it_next != boid_it_last; ++boid_it_next) {
           double diff_x = (boid_it->x - boid_it_next->x);
           double diff_y = (boid_it->y - boid_it_next->y);

    for (; boid_it != boid_it_last; ++boid_it) {
      double diff = norm(b1, *boid_it);
      if (diff < distance_s_) {
        std::vector<BoidState> boids;*/

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

    return VelocityComponents{-s_ * sum_x, -s_ * sum_y};
  }
};
//}
//};

class AllignmentRule {
  int const n_;
  double const a_;

 public:
  AllignmentRule(int const n, double const a) : n_{n}, a_{a} {
    if (a == 1. || a > 1.) {
      throw std::runtime_error{"a must be < than 0"};
    }
  };
  VelocityComponents operator()(std::vector<BoidState> boids,
                                BoidState const& b1) const {
    BoidState sum =
        std::accumulate(boids.begin(), boids.end(), BoidState{0., 0., 0., 0.});
    return VelocityComponents{((sum.v_x - b1.v_x) / (n_ - 1)) * a_,
                              ((sum.v_y - b1.v_y) / (n_ - 1)) * a_};
  }
};

VelocityComponents COM(int const n, std::vector<BoidState> b) {
  auto cboid_it_next = std::next(b.begin());
  // auto cboid_last = std::prev(b_.end());

  BoidState sum =
      std::accumulate(cboid_it_next, b.end(), BoidState{0., 0., 0., 0.});

  double den = 1. / (static_cast<double>(n) - 1.);

  return {sum.x * den, sum.y * den};
}

class CohesionRule {
  int const n_;
  double const cohesion_const_;

 public:
  CohesionRule(int const n, double const c) : n_{n}, cohesion_const_{c} {
    if (n <= 1) {
      throw std::runtime_error{"Error: must be n>1"};
    }
  }

  VelocityComponents operator()(std::vector<BoidState> const& cboids) const {
    VelocityComponents position_of_c = COM(n_, cboids);
    BoidState com{position_of_c.vel_x, position_of_c.vel_y, 0., 0.};
    auto bi = *cboids.begin();
    BoidState result = (com - bi) * cohesion_const_;
    return {result.x, result.y};
  }
};
// dubbio : mettere la vaiabile n solo in boids e non  nelle classi delle
// regole così sono tutte uguali ?
std::vector<BoidState> NeighborsControl(std::vector<BoidState> const& pesci,
                                        BoidState b1, double d) {
  auto p = pesci;
  // auto b1 = *p.begin();
  p.erase(std::remove_if(p.begin(), p.end(),
                         [b1, d](BoidState b) { return (norm(b1, b) > d); }),
          p.end());
  // assert(static_cast<int> pesci.size() == pesci.n());
  return p;
}

void same_position(BoidState const& b1, std::vector<BoidState> boids) {
  for (; boids.begin() != boids.end(); ++boids.begin()) {
    if (b1.x == boids.begin()->x && b1.y == boids.begin()->y) {
      boids.erase(boids.begin());
    }
  }
}

class Boids {
  int const n_;
  double const d_;
  SeparationRule s_;
  AllignmentRule a_;
  CohesionRule c_;
  std::vector<BoidState>
      boids_;  // ho provato a mettere quella n sopra come definizione ma
               // non funziona non ho capito perchè quindi boh -> faccio
               // fatica a definire un numero fisso di entrate del vettore

 public:
  Boids(int const n, double const d, SeparationRule const& s,
        AllignmentRule const& a, CohesionRule const& c)
      : n_{n}, d_{d}, s_{s}, a_{a}, c_{c} {}

  BoidState singleboid(std::vector<BoidState> const& vec, BoidState const& b1,
                       double const delta_t) const {
    VelocityComponents v_old = {b1.v_x, b1.v_y};
    auto v_1 = s_(vec, b1);
    auto v_2 = a_(vec, b1);
    auto v_3 = c_(vec);
    auto v_new = v_old + v_1 + v_2 + v_3;
    return {b1.x + v_new.vel_x * delta_t, b1.y + v_new.vel_y * delta_t,
            v_new.vel_x, v_new.vel_y};
  }

  bool empty() { return boids_.empty(); }
  double distance() const { return d_; }
  std::vector<BoidState> TotalBoids() const { return boids_; }
  int n() const { return n_; }
  double d() const { return d_; }
  SeparationRule s() const { return s_; }
  AllignmentRule a() const { return a_; }
  CohesionRule c() const { return c_; }

  int size() const {
    /*if (boids_.size() > static_cast<size_t>(std::numeric_limits<int>::max()))
    { throw std::overflow_error("size_t value cannot be stored in a variable of
    type int.");
    }*/
    return (static_cast<int>(boids_.size()));
  }

  void push_back(BoidState const& boid) {
    // da mettere controllo che non ci siano boid con la stessa posizione e,
    // fare loop fino a n_ perch avere vettore di quella dimensione e mettere
    // assert su tutto / eccezioni -> comunque questo è l'invariante

    boids_.push_back(boid);
  }

  void evolution(double const delta_t) {
    Boids b{n(), d(), s(), a(), c()};
    std::vector<BoidState> fishes;
    for (auto fish : boids_) {
      auto nearfishes = NeighborsControl(boids_, fish, d_);
      fishes.push_back(b.singleboid(nearfishes, fish, delta_t));
    }
    boids_ = fishes;
  }

  std::vector<BoidState> const& state() const{return boids_;}
};

#endif
