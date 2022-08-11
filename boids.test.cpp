#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boids.hpp"
#include "doctest.h"

<<<<<<< HEAD
<<<<<<< HEAD
TEST_CASE("Check norm2 <= 0") {
  BoidState b1{2., 3., 5., 5.};
  BoidState b2{2., 3., 4., 4.};
  CHECK(((b1.x - b2.x) * (b1.x - b2.x) + (b1.y - b2.y) * (b1.y - b2.y)) == 0.);
}


TEST_CASE("Testing norm function"){
  BoidState b1{2., 3., 5., 5.};
  BoidState b2{2., 3., 4., 4.};
  BoidState b3{5., 6., 0., 0.};
  CHECK(norm(b1, b3) == doctest::Approx(4.24).epsilon(0.01));
  CHECK(norm(b1, b2) == 0.);
}


TEST_CASE("Testing operators") {
    SUBCASE("Check operator == on identical boids") {
    BoidState b1{2., 3., 4., 0.};
    BoidState b2{2., 3., 4., 0.};
    CHECK((b1 == b2) == true);
  }

  SUBCASE("Check operator == on different boids") {
    BoidState b1{2., 2., 4., 0.};
    BoidState b2{2., 3., 4., 0.};
    CHECK((b1 == b2) == false);
  }

  SUBCASE("Check operator != on different boids") {
    BoidState b1{2., 5., 3., 1.};
    BoidState b2{1., 3., 4., 0.};
    CHECK((b1 != b2) == true);
  }

  SUBCASE("Check operator != on identical boids") {
    BoidState b1{2., 3., 4., 0.};
    BoidState b2{2., 3., 4., 0.};
    CHECK((b1 != b2) == false);
  }

  SUBCASE("Check addition with two points") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{4., 3., 2., 1.};
    CHECK((b1 + b2).x == 5.);
    CHECK((b1 + b2).y == 5.);
    CHECK((b1 + b2).v_x == 5.);
    CHECK((b1 + b2).v_y == 5.);  
  }
  SUBCASE("Check addition with three points") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{4., 3., 2., 1.};
    BoidState b3{5., 9., 2., 6.};
    CHECK((b1 + b2 + b3).x == 10.);
    CHECK((b1 + b2 + b3).y == 14.);
    CHECK((b1 + b2 + b3).v_x == 7.);
    CHECK((b1 + b2 + b3).v_y == 11.);
  }
  SUBCASE("Check addition with negative values") {
    BoidState b1{-1., 2., 3., 4.};
    BoidState b2{4., -3., 2., -1.};
    CHECK((b1 + b2).x == 3.);
    CHECK((b1 + b2).y == -1.);
    CHECK((b1 + b2).v_x == 5.);
    CHECK((b1 + b2).v_y == 3.);
  }

  SUBCASE("Check addition with zero") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{0., 0., 0., 0.};
    CHECK(((b1 + b2) == b1));
  }

  SUBCASE("Check difference with zero") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{0., 0., 0., 0.};
    CHECK(((b1 - b2) == b1));
  }
  SUBCASE("Check difference with two points ") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{4., 3., 2., 1.};
    CHECK((b1 - b2).x == -3.);
    CHECK((b1 - b2).y == -1.);
    CHECK((b1 - b2).v_x == 1.);
    CHECK((b1 - b2).v_y == 3.);
  }
  SUBCASE("Check difference with negative numbers") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{-1., -3., -2., -4.};
    CHECK((b1 - b2).x == 2.);
    CHECK((b1 - b2).y == 5.);
    CHECK((b1 - b2).v_x == 5.);
    CHECK((b1 - b2).v_y == 8.);
  }
  SUBCASE("Check difference which is zero") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{1., 2., 3., 4.};
    CHECK((b1 - b2).x == 0.);
    CHECK((b1 - b2).y == 0.);
    CHECK((b1 - b2).v_x == 0.);
    CHECK((b1 - b2).v_y == 0.);
  }
  /*SUBCASE("Check denominator == 0") {
    BoidState b1{2., 0., 5., 5.};
    BoidState b2{2., 3., 4., 0.};
    CHECK_FALSE((b1/b2) == 0.);
  }
  SUBCASE("Check multiplication with zero") {
    BoidState b1{2., 0., 5., 5.};
    BoidState b2{2., 3., 4., 0.};
    CHECK((b1 * b2) == {4., 0., 20., 0});
  }*/ //commentati perché non credo ci serviranno
}
<<<<<<< HEAD

TEST_CASE("Testing Rules"){

  SUBCASE("Check SeparationRule ")
   BoidState b1{2., 2., 5., 5.};
   BoidState b2{2., 3., 4., 0.};
   int n_ {2};
   double const separation_const_ {0.5};
   double const distance_s_ {2.};
   CHECK()
   
   
   
  SUBCASE("Check SeparationRule with the same boid ")
   BoidState b1{2., 2., 5., 5.};
   BoidState b2{2., 2., 5., 5.};
   int n {2};
   double const separation_const_ {0.5};
   double const distance_s {2.};
   CHECK_FALSE(v_1 == 0.);         
  



SUBCASE("Check SeparationRule with norm > distance_s ")
   BoidState b1{1., 2., 7., 5.};
   BoidState b2{2., 1.5, 5., 4.};
   int n {2};
   double const separation_const_ {0.5};
   double const distance_s {2.};
   CHECK_FALSE(v_1 == doctest::Approx(0.6));  
   


SUBCASE("Check AllignementRule, boids have the same velocity")
   BoidState b1{8., 2., 5., 5.};
   BoidState b2{2., 5., 5., 5.};
   int n {2};
   double const allignement_const_ {0.7};
   CHECK(v_2 ==);      //da implementare




SUBCASE("Check CohesionRule, two boids have the same position")
   BoidState b1{2., 2., 5., 5.};
   BoidState b2{2., 2., 4., 0.};
   int n {2};
   double const allignement_const_ {1.};
   CHECK(v_3 == );      //da implementare
 



=======
TEST_CASE("Testing alignment rule") {
  SUBCASE("General tests") {
  BoidState b1 = {0.,0.,2.,3.};
  BoidState b2 = {0.,0.,5.,1.};
  BoidState b3 = {0.,0.,-2.,3.};
  std::vector<BoidState> a{b1,b2,b3};
  BoidState b = {0.,0.,1.,-1};
  AllignmentRule ar{4, 0.8};
  CHECK(ar(b, a).vel_x == doctest::Approx(1.064).epsilon(0.01));
  CHECK(ar(b, a).vel_y == doctest::Approx(2.136).epsilon(0.01));
  BoidState b_ = {0.,0.,5.,7.};
  CHECK(ar(b_ , a).vel_x == 0.);
  CHECK(ar(b_ , a).vel_y == 0.);
  }
  SUBCASE("a greater than 1"){
  CHECK_THROWS(AllignmentRule {5,1.2});
  }
  /*SUBCASE("Trying to break the code"){
    //non ho idee per ora
  }*/
>>>>>>> 26c607fe6206e988f3b73739e00338be59cd917e
}