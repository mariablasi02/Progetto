#include <algorithm>
#include <iostream>
#include <numeric>

#include "boids.hpp"

int main() {
  BoidState n1{0., 0., 0., 0.};
  BoidState n2{0., 10., 0., 1.};
  BoidState n3{0., 30., 0., 1.};
  SeparationRule s{5, 3.};
  AlignmentRule a{0.5};
  CohesionRule c{3};
  Boids bb{3, 60., s, a, c};
  bb.push_back(n1);
  bb.push_back(n2);
  bb.push_back(n3);
  bb.evolution(0.2);
  std::cout << "x: " << bb.TotalBoids()[0].x << "y: " << bb.TotalBoids()[0].y
            << " v_x: " << bb.TotalBoids()[0].v_x
            << " v_y: " << bb.TotalBoids()[0].v_y << '\n';
  std::cout << "x: " << bb.TotalBoids()[1].x << "y: " << bb.TotalBoids()[1].y
            << " v_x: " << bb.TotalBoids()[1].v_x
            << " v_y: " << bb.TotalBoids()[1].v_y << '\n';
  std::cout << "x: " << bb.TotalBoids()[2].x << "y: " << bb.TotalBoids()[2].y
            << " v_x: " << bb.TotalBoids()[2].v_x
            << " v_y: " << bb.TotalBoids()[2].v_y << '\n';
  std::cout<< norm(bb.TotalBoids()[0] , bb.TotalBoids()[1]) << '\n';
  std::cout<< norm(bb.TotalBoids()[0], bb.TotalBoids()[2])<< '\n';
  std::cout<< norm(bb.TotalBoids()[2], bb.TotalBoids()[1]) << '\n';
  std::cout << velocity_norm(bb.TotalBoids()[0]) << '\n';
  std::cout << velocity_norm(bb.TotalBoids()[1]) << '\n';
  std::cout << velocity_norm(bb.TotalBoids()[2]) << '\n';
}
