#include <iostream>

#include "boids.hpp"

void check_borders(Boids& boids) {
  int i = 0;
  /* auto boidscopy = boids.TotalBoids();
  for (auto& b : boidscopy) {
    if (b.x < 0.f) {
    } else if (b.x >= 1179) {
    } else if ()b.y  {
    } else {
    }
    ++i
  } */
}

int main() {
  BoidState b1{0., 3., 4., 2.};
  BoidState b2{6., 1., -1, 1.};
  BoidState b3{4., 3., 4., 1.};
  std::vector<BoidState> v{b1, b2, b3};
  SeparationRule s{5., 3.};
  AllignmentRule a{0.5};
  CohesionRule c{3.};
  // auto vector = NeighborsControl(v, b1, 6.);
  // std::cout<< vector.size() << '\n';
  /* std::cout<< "Components x and y of v1: " << s(v, b1).val_x << "x" << s(v,
  b1).val_y << "y" << '\n'; std::cout<< "Components x and y of v2: " << a(v,
  b1).val_x << "x" << a(v, b1).val_y << "y" << '\n'; std::cout<< "Components x
  and y of v3: " << c(v, b1).val_x << "x" << c(v, b1).val_y << "y" << '\n'; */
  Boids flock{3, 2., s, a, c};
  (flock.TotalBoids()).push_back(b1);
  (flock.TotalBoids()).push_back(b1);
  (flock.TotalBoids()).push_back(b1);
  auto v = flock.TotalBoids();
  std::cout << v[0].x;
}