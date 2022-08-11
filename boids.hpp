#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <vector>
// manca il controllo degli errori su tutto

struct BoidState {
  double x{};
  double y{};
  double v_x{};
  double v_y{};
<<<<<<< HEAD
  // BoidState(double x, double y, double vx, double vy) : x{x}, y{y}, v_x{vx},
  // v_y{vy} {};
=======
  BoidState(double x, double y, double vx, double vy)
      : x{x}, y{y}, v_x{vx}, v_y{vy} {};
>>>>>>> 26c607fe6206e988f3b73739e00338be59cd917e
  BoidState& operator+=(BoidState const& other) {
    x += other.x;
    y += other.y;
    v_x += other.v_x;
    v_y += other.v_y;
    return *this;
  }
<<<<<<< HEAD
  
=======
  double norm(BoidState const& other) {
    auto result = (x - other.x) * (x - other.x) + (y - other.y) * (y - other.y);
    assert(!(result < 0));
    return std::sqrt(result);
  }
>>>>>>> 26c607fe6206e988f3b73739e00338be59cd917e
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
  return {b1.x - b2.x, b1.y- b2.y ,b1.v_x - b2.v_x, b1.v_y - b2.v_y};
  
}
BoidState operator*(BoidState const& b1, BoidState const& b2) {
   return {b1.x * b2.x, b1.y* b2.y ,b1.v_x * b2.v_x, b1.v_y * b2.v_y};
 
}
BoidState operator/(BoidState const& b1, BoidState const& b2) {
  if (b2.x == 0 || b2.y == 0 || b2.v_x == 0 || b2.v_y == 0) {
    throw std::runtime_error{"Denominator is zero"};
     return {b1.x / b2.x, b1.y / b2.y ,b1.v_x / b2.v_x, b1.v_y / b2.v_y};
  }
 
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

<<<<<<< HEAD

// idea : si potrebbero implementare le classi delle regole già con dei vettori
// che restituiscono le regole -> dopo mando audio

=======
>>>>>>> 26c607fe6206e988f3b73739e00338be59cd917e
class SeparationRule {
  int const n_;
  double const separation_const_;

  double const distance_s_;
  // valutare un valore che viene deciso da noi
<<<<<<< HEAD

 public:
  SeparationRule(int const n, double const s, double d_s)
      : n_{n}, separation_const_{s}, distance_s{d_s} {
    if (n_ <= 1) {
      throw std::runtime_error{"Number of boids must be >1"};
    }
  }

  std::vector<BoidState> boids;

  auto boid_it = boids.begin();
  auto boid_it_next = std::next(boids.begin());
  auto boid_it_last = std::prev(boids.end());


  std::vector<double> boidsdiff_x;
  std::vector<double> boidsdiff_y;

  for (; boid_it_next != boid_it_last; ++boid_it_next) {
    double diff_x = (boid_it->x - boid_it_next->x);
    double diff_y = (boid_it->y - boid_it_next->y);

    boidsdiff_x.push_back(diff_x);
    boidsdiff_y.push_back(diff_y);
=======

 public:
  SeparationRule(int const n, double const s, double d_s)
      : n_{n}, separation_const_{s}, distance_s_{d_s} {
    if (n_ <= 1) {
      throw std::runtime_error{"Number of boids must be >1"};
    }
  }
  auto operator()(std::vector<BoidState> boids, BoidState const& b1) const {
    auto boid_it = boids.begin();
    auto boid_it_next = std::next(boids.begin());
    auto boid_it_last = std::prev(boids.end());

    for (; boid_it_next != boid_it_last; ++boid_it_next) {
      double diff = norm(*boid_it, *boid_it_next);
      if (diff < distance_s_) {
        std::vector<BoidState> boids;

        std::vector<double> boidsdiff_x;
        std::vector<double> boidsdiff_y;

        for (; boid_it_next != boid_it_last; ++boid_it_next) {
          double diff_x = (boid_it->x - boid_it_next->x);
          double diff_y = (boid_it->y - boid_it_next->y);

          boidsdiff_x.push_back(diff_x);
          boidsdiff_y.push_back(diff_y);
        }

        double sum_x =
            std::accumulate(boidsdiff_x.begin(), boidsdiff_x.end(), 0.);
        double sum_y =
            std::accumulate(boidsdiff_y.begin(), boidsdiff_y.end(), 0.);

        return VelocityComponents{-separation_const_ * sum_x,
                                  -separation_const_ * sum_y};
      }
    }
  }
};

class AllignmentRule {
  int const n_;
  double const a_;

 public:
  AllignmentRule(int const n, double const a) : n_{n}, a_{a} {
    if (a == 1. || a > 1.) {
      throw std::runtime_error{"a must be < than 0"};
    }
  };
  VelocityComponents operator()(BoidState const& b1,
                                std::vector<BoidState> boids) const {
    // boids = std::remove_if(boids.begin(), boids.end(), [b1, double
    // d](BoidState b){return norm(b1, b) > d;}); // work in progress
    BoidState sum =
        std::accumulate(boids.begin(), boids.end(), BoidState{0., 0., 0., 0.});
    return VelocityComponents{((sum.v_x - b1.v_x) / (n_ - 1)) * a_,
                              ((sum.v_y - b1.v_y) / (n_ - 1)) * a_};
>>>>>>> 26c607fe6206e988f3b73739e00338be59cd917e
  }

  double sum_x = std::accumulate(boidsdiff_x.begin(), boidsdiff_x.end(), 0.);
  double sum_y = std::accumulate(boidsdiff_y.begin(), boidsdiff_y.end(), 0.);

  auto operator()(std::vector<BoidState> boids, BoidState const& b1) const {
    for (; boid_it_next != boid_it_last; ++boid_it_next) {
      double diff = norm(*boid_it, *boid_it_next);
      if (diff < distance_s_) {
        return VelocityComponents{-separation_const_ * sum_x,
                                  -separation_const_ * sum_y};
      } 
    }
  }
}


/*AllignmentRule(int const n, double const a) : n_{n}, allignment_const_{a} {}

VelocityComponents operator()(BoidState const& b1,
                              BoidState const& b2) const {
  return {};
};

<<<<<<< HEAD
class CohesionRule {
int const n_;
double const cohesion_const_;
=======
/*class CohesionRule {
  int const n_;
  double const cohesion_const_;
>>>>>>> 26c607fe6206e988f3b73739e00338be59cd917e

public:
CohesionRule(int const n, double const c) : n_{n}, cohesion_const_{c} {}


  VelocityComponents COM(int const n_, BoidState const& b1) { 

   
    return {}; }

  VelocityComponents operator()(BoidState const& b1,
                                BoidState const& b2) const {

    
    return {};
  }

VelocityComponents COM(int const n_, BoidState const& b1) { return {}; }

VelocityComponents operator()(BoidState const& b1,
                              BoidState const& b2) const {
  return {};
}

};

// dubbio : mettere la vaiabile n solo in boids e non  nelle classi delle regole
// così sono tutte uguali ?

class Boids {
int const n_;
double const distance_;
SeparationRule s_;
AllignmentRule a_;
CohesionRule c_;

std::vector<BoidState>
    boids_;  // ho provato a mettere quella n sopra come definizione ma
             // non funziona non ho capito perchè quindi boh -> faccio
             // fatica a definire un numero fisso di entrate del vettore
BoidState solve(BoidState const& b1, VelocityComponents const& v1,
                VelocityComponents const& v2, VelocityComponents const& v3,
                double const delta_t) const {
  return {};
}

public:
Boids(int const n, double const d, SeparationRule const& s,
      AllignmentRule const& a, CohesionRule const& c)
    : n_{n}, distance_{d}, s_{s}, a_{a}, c_{c} {}

bool empty() { return boids_.empty(); }

std::size_t size() const { return boids_.size(); }

void push_back(BoidState const& boid) {
  // da mettere controllo che non ci siano boid con la stessa posizione e,
  // fare loop fino a n_ perch avere vettore di quella dimensione e mettere
  // assert su tutto / eccezioni -> comunque questo è l'invariante

  boids_.push_back(boid);
}

<<<<<<< HEAD
void evolution(double const delta_t) {}

=======
  void evolution(double const delta_t);

  std::vector<BoidState> const &state() const;
};*/
>>>>>>> 26c607fe6206e988f3b73739e00338be59cd917e

  std::vector<BoidState> const& state() const {
    return boids_;
  }  // non capisco perchè dia errore qui

  std::vector<BoidState> const& state() const;

std::vector<BoidState> const& state() const {
  return boids_;
}  // non capisco perchè dia errore qui

};
*/
#endif
