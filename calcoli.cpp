#include <iostream>

#include "boids.hpp"

int main() {
  
  BoidState b1{3., 1., 0.3, 2.0};
  BoidState b2{2., 1., 1.7, -0.7};
  BoidState b3{1., 4., -0.5, 0.4};
  SeparationRule s{0.2, 20};
  AlignmentRule a{0.3};
  CohesionRule c{1.3};
  std::vector<BoidState> v{b1, b2, b3};
  auto vector = NeighborsControl(v, b2, 40.);
  std::cout << vector.size() << '\n';
  std::cout << "Components x and y of v1: " << s(vector, b2).val_x << "x"
            << s(v, b1).val_y << "y" << '\n';
  std::cout << "Components x and y of v2: " << a(vector, b2).val_x << "x"
            << a(vector, b1).val_y << "y" << '\n';
  std::cout << "Components x and y of v3: " << c(vector, b2).val_x << "x"
            << c(vector, b1).val_y << "y" << '\n';

}