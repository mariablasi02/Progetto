#include <algorithm>
#include <iostream>
#include <numeric>

#include "boids.hpp"

int main() {
  BoidState b1{3., 1., 0.3, 2.0};
  BoidState b2{2., 1., 1.7, -0.7};
  BoidState b3{1., 4., -0.5, 0.4};
  SeparationRule s{0.2, 20};
  AlignmentRule a{0.3};
  CohesionRule c{1.3};
  Boids boid{3, 40., s, a, c};
  boid.push_back(b1);
  boid.push_back(b2);
  boid.push_back(b3);

  /* std::cout << s(boid.TotalBoids(), b1).val_x << " "
            << s(bb.TotalBoids(), b1).val_y << '\n';
  std::cout << a(bb.TotalBoids(), b1).val_x << " "
            << a(bb.TotalBoids(), b1).val_y << '\n';
  std::cout << c(bb.TotalBoids(), b1).val_x << " "
            << c(bb.TotalBoids(), b1).val_y << '\n'; */

  std::cout << (boid.singleboid(boid.TotalBoids(), b2, 0.5)).v_x << '\n';
  std::cout << (boid.singleboid(boid.TotalBoids(), b2, 0.5)).v_y << '\n';
  std::cout << (boid.singleboid(boid.TotalBoids(), b1, 0.5)).v_x << '\n';
  std::cout << (boid.singleboid(boid.TotalBoids(), b1, 0.5)).v_y << '\n';
}
