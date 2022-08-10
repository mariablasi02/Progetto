#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

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
};

BoidState operator==(BoidState const& b1, BoidState const& b2) {
  return {b1.x == b2.x, b1.y == b2.y, b1.v_x == b2.v_x, b1.v_y == b2.v_y};
}

BoidState operator!=(BoidState const& b1, BoidState const& b2) {
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
  auto result =
      std::abs((b1.x - b2.x) * (b1.x - b2.x) + (b1.y - b2.y) * (b1.y - b2.y));
  assert(result > 0);  // non so se abbia senso ora mantenere questo assert
                       // visto che ho messo ||
  return std::sqrt(result);
}

// nelle classi private si sceglie di mantenere _ alla fine dei data members

struct VelocityComponents {
  double vel_x;
  double vel_y;
};

// idea : si potrebbero implementare le classi delle regole già con dei vettori
// che restituiscono le regole -> dopo mando audio

class SeparationRule {
  int const n_;
  double const separation_const_;
  double const distance_s_;

 public:
  // valutare un valore che viene deciso da noi->
  // valutare se mettere pubblico o privato

  SeparationRule(int const n, double const s, double const ds)
      : n_{n}, separation_const_{s}, distance_s_{ds} {}

  VelocityComponents operator()(BoidState const& b1,
                                BoidState const& b2) const {
    return {};
  }
};

class AllignmentRule {
  int const n_;
  double const allignment_const_;

 public:
  AllignmentRule(int const n, double const a) : n_{n}, allignment_const_{a} {}

  VelocityComponents operator()(BoidState const& b1,
                                BoidState const& b2) const {
    return {};
  }
};

class CohesionRule {
  int const n_;
  double const cohesion_const_;

 public:
  CohesionRule(int const n, double const c) : n_{n}, cohesion_const_{c} {}

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

  void evolution(double const delta_t) {}

  std::vector<BoidState> const& state() const {
    return boids_;
  }  // non capisco perchè dia errore qui
};

#endif
