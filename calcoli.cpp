#include <iostream>

#include "boids.hpp"

auto sum ( BoidState a, BoidState b, BoidState c, BoidState d){
 auto sumx = a.x +b.x + c.x+ d.x;
 auto sumy = a.y +b.y + c.y+d.y;
 auto sumvx = a.v_x +b.v_x + c.v_x+ d.v_x;
 auto sumvy= a.v_y +b.v_y + c.v_y+d.v_y;

 return Components{sumvx, sumvy};
}


int main() {
 CohesionRule c1{-1.7};
    BoidState b1{1.3, 2., 3.6, 4.};
    BoidState b2{2., 3., 4., 5.};
    BoidState b3{-1., -1.3, -1., -1.7};
    BoidState b4{0., -1., 3., -2.};
    std::vector<BoidState> v1{b1, b2, b3, b4};

  std::cout << "result: " << COM(v1, b3).val_x << '\n';
  std::cout << "result: " << COM(v1, b3).val_y << '\n';
  std::cout << "result: " << COM(v1,b4).val_x << '\n';
  std::cout << "result: " << COM(v1,b4).val_y << '\n';


  //std::cout << "result: " << norm(b2, b3) << '\n';
}
