#include "boids.hpp"
#include <iostream>


void check_borders(Boids& boids){
    int i = 0;
    auto boidscopy = boids.TotalBoids();
    for(auto& b : boidscopy){
        if (b.x == 0.f ){
      b.x = 1178;
    }
    if (b.x == 1179 ){
      b.x = 1;
    }
    /*if (b.y  ){
      rec.setPosition(rec.getPosition().x, 0.f);
    }
    if (rec.getPosition().y + rec.getGlobalBounds().height >691 ){
      rec.setPosition(rec.getPosition().x, 691.f- rec.getGlobalBounds().height);
    }*/
    boids.TotalBoids() = boidscopy;
      
      ++i;  
  
    }
}
int main() {
    BoidState b1{0., 3., 4., 2.};
    BoidState b2{6., 1., -1, 1.};
    BoidState b3{4., 3., 4., 1.};
    SeparationRule s{5., 3.};
    AllignmentRule a{0.5};
    CohesionRule c{3.};
    Boids flock{3, 5., s, a, c};
    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);

    auto v = flock.TotalBoids();
    std::cout<< v[0].x << '\n';
    
}
/*switch(static_cast<int>(b.y)) {
        case 0:
        boids.TotalBoids()[i].y = 690;      
        break;
        case 691:
        boids.TotalBoids()[i].y = 1.; 
        break;
      } 
    switch(static_cast<int>(b.x)) {
        double result;
      case 0:
      boids.TotalBoids()[i].x = 1178;
      break;
      case 1179:
      boids.TotalBoids()[i].x = 1.;
    }*/