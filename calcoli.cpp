#include "boids.hpp"
#include <iostream>
int main() {
    BoidState b1{2., 3., 4., 2.};
    BoidState b2{6., 1., -1, 1.};
    BoidState b3{4., 3., 4., 1.};
    std::vector<BoidState> v{b1, b2, b3};
    SeparationRule s{5., 3.};
    AllignmentRule a{0.5};
    CohesionRule c{3.};
    auto vector = NeighborsControl(v, b1, 6.);
    std::cout<< vector.size() << '\n';
    std::cout<< "Components x and y of v1: " << s(v, b1).val_x << "x" << s(v, b1).val_y << "y" << '\n';
    std::cout<< "Components x and y of v2: " << a(v, b1).val_x << "x" << a(v, b1).val_y << "y" << '\n';
    std::cout<< "Components x and y of v3: " << c(v, b1).val_x << "x" << c(v, b1).val_y << "y" << '\n';
}


