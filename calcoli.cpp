#include <iostream>

#include "boids.hpp"

int main() {
    BoidState b1{0.3, 1., 2., 0.5};
    BoidState b2{0.02, 1., 0.1, -2};
    BoidState b3{5, 4., -1, -0.6};
    SeparationRule s{3, 5};
    AlignmentRule a{0.3};
    CohesionRule c{0.9};
    Boids Nemo{3, 300., s, a, c};
    Nemo.push_back(b1);
    Nemo.push_back(b2);
    Nemo.push_back(b3);
    Nemo.evolution(0.5);
    std::cout <<"x: " << Nemo.TotalBoids()[0].x << '\t' << "y: " << Nemo.TotalBoids()[0].y << '\t' << "v_x: " << Nemo.TotalBoids()[0].v_x << '\t' << "v_y: " << Nemo.TotalBoids()[0].v_y << '\n';
    std::cout <<"x: " << Nemo.TotalBoids()[1].x << '\t' << "y: " << Nemo.TotalBoids()[1].y << '\t' << "v_x: " << Nemo.TotalBoids()[1].v_x << '\t' << "v_y: " << Nemo.TotalBoids()[1].v_y << '\n';
    std::cout <<"x: " << Nemo.TotalBoids()[2].x << '\t' << "y: " << Nemo.TotalBoids()[2].y << '\t' << "v_x: " << Nemo.TotalBoids()[2].v_x << '\t' << "v_y: " << Nemo.TotalBoids()[2].v_y << '\n';
  
}