#include <iostream>

#include "boids.hpp"

/*void check_borders(Boids& boids){
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
/* boids.TotalBoids() = boidscopy;

    ++i;

  }
}*/
/*int main() {
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
    std::cout<< v[0].x << '\n';*/
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

/*auto sum(BoidState a, BoidState b, BoidState c, BoidState d) {
  auto sumx = a.x + b.x + c.x + d.x;
  auto sumy = a.y + b.y + c.y + d.y;
  auto sumvx = a.v_x + b.v_x + c.v_x + d.v_x;
  auto sumvy = a.v_y + b.v_y + c.v_y + d.v_y;

  return Components{sumvx, sumvy};
}

CohesionRule c1{-1.7};
BoidState b1{1.3, 2., 3.6, 4.};
BoidState b2{2., 3., 4., 5.};
BoidState b3{-1., -1.3, -1., -1.7};
BoidState b4{0., -1., 3., -2.};
std::vector<BoidState> v1{b1, b2, b3, b4};

std::cout << "result: " << COM(v1, b3).val_x << '\n';
std::cout << "result: " << COM(v1, b3).val_y << '\n';
std::cout << "result: " << COM(v1, b4).val_x << '\n';
std::cout << "result: " << COM(v1, b4).val_y << '\n';

// std::cout << "result: " << norm(b2, b3) << '\n';
}*/
#include <iostream>

#include "boids.hpp"
int main() {
    BoidState b1{3., 1., 275., 100.};
    BoidState b2{2., 1., 78., 420};
    BoidState b3{1., 4., 190, 600};
    SeparationRule s{0.2, 20};
    AlignmentRule a{0.3};
    CohesionRule c{1.3};
  std::vector<BoidState> v{b1, b2, b3};
  auto vector = NeighborsControl(v, b2, 40.);
  std::cout << vector.size() << '\n';
  std::cout << "Components x and y of v1: " << s(v, b1).val_x << "x"
            << s(v, b1).val_y << "y" << '\n';
  std::cout << "Components x and y of v2: " << a(vector, b1).val_x << "x"
            << a(vector, b1).val_y << "y" << '\n';
  std::cout << "Components x and y of v3: " << c(vector, b1).val_x << "x"
            << c(vector, b1).val_y << "y" << '\n';
}