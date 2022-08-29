#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cassert>
#include <stdexcept>
#include <vector>

struct BoidState {
  double x{};
  double y{};
  double v_x{};
  double v_y{};

  BoidState& operator+=(BoidState const& other);

  BoidState& operator*=(double const other);

  BoidState& operator*=(BoidState const& other);
};

bool operator==(BoidState const& b1, BoidState const& b2);

bool operator!=(BoidState const& b1, BoidState const& b2);

BoidState operator+(BoidState const& b1, BoidState const& b2);

BoidState operator-(BoidState const& b1, BoidState const& b2);

BoidState operator*(BoidState const& b1, double const d);

BoidState operator*(BoidState const& b1, BoidState const& b2);

double norm(BoidState const& b1, BoidState const& b2);

double velocity_norm(BoidState const& b);

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

std::vector<BoidState> neighborscontrol(std::vector<BoidState> const& pesci,
                                        BoidState const& b1, double const d);

bool check_ownership(std::vector<BoidState> const& cont, BoidState const& c);

class SeparationRule {
  double const s_;
  double const distance_s_;

 public:
  SeparationRule(double const s, double d_s) : s_{s}, distance_s_{d_s} {}
  Components operator()(std::vector<BoidState> const& b,
                        BoidState const& b1) const;
};

class AlignmentRule {
  double const a_;

 public:
  AlignmentRule(double const a) : a_{a} {
    if (a == 1. || a > 1.) {
      throw std::runtime_error{"a must be < than 1"};
    }
  }

  double get_a() const;

  Components operator()(std::vector<BoidState> const& boids,
                        BoidState const& b1) const;
};

Components centre_of_mass(std::vector<BoidState> const& vec,
                          BoidState const& b1);

class CohesionRule {
  double const cohesion_const_;

 public:
  CohesionRule(double const c) : cohesion_const_{c} {}

  Components operator()(std::vector<BoidState> const& cboids,
                        BoidState const& b1) const;
};

bool same_pos_check(BoidState const& b1, std::vector<BoidState> const& boids);

bool same_pos_check(std::vector<BoidState> const& boid);

std::vector<BoidState> velocity_limit(std::vector<BoidState>& boidsvec);

std::vector<BoidState> borders(std::vector<BoidState>& v);

class Boids {
  int const n_;
  double const d_;
  SeparationRule const s_;
  AlignmentRule const a_;
  CohesionRule const c_;
  std::vector<BoidState> boids_;

 public:
  Boids(int const n, double const d, SeparationRule const& s,
        AlignmentRule const& a, CohesionRule const& c)
      : n_{n}, d_{d}, s_{s}, a_{a}, c_{c} {
    assert(n > 1);
    assert(a_.get_a() < 1.);
  }

  BoidState singleboid(std::vector<BoidState> const& vec, BoidState const& b1,
                       double const delta_t) const;

  std::vector<BoidState> totalboids() const;

  void pushback(BoidState const& boid);

  void evolution(double const delta_t);

  void setvector(std::vector<BoidState> const& b);
};

struct Stats {
  double mean_distance;
  double std_distance;
  double mean_speed;
  double std_speed;
};
Stats statistic(Boids& b, double const delta_t);
std::string state(Boids& b, double const delta_t);

#endif
