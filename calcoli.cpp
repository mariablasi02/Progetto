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
  Boids boid{3, 60., s, a, c};
  boid.pushback(n1);
  boid.pushback(n2);
  boid.pushback(n3);

  boid.evolution(0.2);
  /* std::cout << "v_x " << boid.TotalBoids()[0].v_x << " v_y "
            << boid.TotalBoids()[0].v_y << '\n';
  std::cout << "v_x " << boid.TotalBoids()[1].v_x << " v_y "
            << boid.TotalBoids()[1].v_y << '\n';
  std::cout << "v_x " << boid.TotalBoids()[2].v_x << " v_y "
            << boid.TotalBoids()[2].v_y << '\n'; */
  std::cout << norm(boid.TotalBoids()[0],boid.TotalBoids()[1]) << '\n';
  std::cout << norm(boid.TotalBoids()[1],boid.TotalBoids()[2]) << '\n';
  std::cout << norm(boid.TotalBoids()[0],boid.TotalBoids()[2]) << '\n';
  /* std::cout << s(boid.TotalBoids(), b1).val_x << " "
            << s(bb.TotalBoids(), b1).val_y << '\n';
  std::cout << a(bb.TotalBoids(), b1).val_x << " "
            << a(bb.TotalBoids(), b1).val_y << '\n';
  std::cout << c(bb.TotalBoids(), b1).val_x << " "
            << c(bb.TotalBoids(), b1).val_y << '\n'; */
  /*
    std::cout << (boid.singleboid(boid.TotalBoids(), n1, 0.5)).v_x << '\n';
    std::cout << (boid.singleboid(boid.TotalBoids(), n1, 0.5)).v_y << '\n';
    std::cout << (boid.singleboid(boid.TotalBoids(), n2, 0.5)).v_x << '\n'; */
}
