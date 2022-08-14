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
#include <typeinfo>
#include <vector>

// manca il controllo degli errori su tutto

struct BoidState {
  double x{};
  double y{};
  double v_x{};
  double v_y{};

  BoidState& operator+=(BoidState const& other);

  BoidState& operator*=(double const other);

  BoidState& operator*=(BoidState const other);
};

bool operator==(BoidState const& b1, BoidState const& b2);

bool operator!=(BoidState const& b1, BoidState const& b2);

BoidState operator+(BoidState const& b1, BoidState const& b2);

BoidState operator-(BoidState const& b1, BoidState const& b2);

BoidState operator*(BoidState const& b1, double const d);

BoidState operator*(BoidState const& b1, BoidState const& b2);

double norm(BoidState const& b1, BoidState const& b2);

struct Components {
  double val_x;
  double val_y;

  Components& operator+=(Components const& other);

  Components& operator*=(double const d);
};

Components operator+(Components const& c1, Components const& c2);

Components operator-(Components const& c1, Components const& c2);

Components operator*(Components const& c1, double const d);

bool operator==(Components const& c1, Components const& c2);

bool operator!=(Components const& c1, Components const& c2);

int size(std::vector<BoidState> const& v);

std::vector<BoidState> NeighborsControl(std::vector<BoidState> const& pesci,
                                        BoidState b1, double d);

class SeparationRule {
  double const s_;
  double const distance_s_;
  // valutare un valore che viene deciso da noi

 public:
  SeparationRule(double const s, double d_s) : s_{s}, distance_s_{d_s} {}
  Components operator()(std::vector<BoidState> const& b,
                        BoidState const& b1) const;
};

bool check_ownership(std::vector<BoidState> const& cont, BoidState const& c);

class AllignmentRule {
  double const a_;

 public:
  AllignmentRule(double const a) : a_{a} {
    if (a == 1. || a > 1.) {
      throw std::runtime_error{"a must be < than 1"};
    }
  }

  double get_a() const;

  Components operator()(std::vector<BoidState> boids,
                        BoidState const& b1) const;
};

Components COM(std::vector<BoidState> const& vec, BoidState const& b1);

class CohesionRule {
  double const cohesion_const_;

 public:
  CohesionRule(double const c) : cohesion_const_{c} {}

  Components operator()(std::vector<BoidState> const& cboids,
                        BoidState const& b1) const;
};

bool same_pos_check(BoidState const& b1, std::vector<BoidState> boids);

void same_position(BoidState const& b1, std::vector<BoidState> boids);

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
                       double const delta_t) const;

  std::vector<BoidState> TotalBoids() const;

  int n() const;

  double d() const;

  SeparationRule s() const;

  AllignmentRule a() const;

  CohesionRule c() const;

  void push_back(BoidState const& boid);

  void evolution(double const delta_t);

  void setvector(std::vector<BoidState> const& b);
};

void state(Boids& b, double const delta_t);

#endif
