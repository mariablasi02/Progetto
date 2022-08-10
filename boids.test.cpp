#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boids.hpp"
#include "doctest.h"

<<<<<<< HEAD
TEST_CASE("Check norm2 <= 0") {
  BoidState b1{2., 3., 5., 5.};
  BoidState b2{2., 3., 4., 4.};
  CHECK(((b1.x - b2.x) * (b1.x - b2.x) + (b1.y - b2.y) * (b1.y - b2.y)) == 0.);
}

=======
>>>>>>> 087ecd171d1ede81d14468e2358f5b1dcf0a6f2a
TEST_CASE("Testing operators") {
  SUBCASE("Check norm2 <= 0") {
    BoidState b1{2., 3., 5., 5.};
    BoidState b2{2., 3., 4., 4.};
    CHECK(((b1.x - b2.x) * (b1.x - b2.x) + (b1.y - b2.y) * (b1.y - b2.y)) ==
          0.);
  }
  SUBCASE("Check operator == on identical boids") {
    BoidState b1{2., 3., 4., 0.};
    BoidState b2{2., 3., 4., 0.};
    CHECK(b1 == b2);
  }

  SUBCASE("Check operator == on different boids") {
    BoidState b1{2., 2., 4., 0.};
    BoidState b2{2., 3., 4., 0.};
    CHECK_FALSE(b1 == b2);
  }

  SUBCASE("Check operator != on different boids") {
    BoidState b1{2., 5., 3., 1.};
    BoidState b2{1., 3., 4., 0.};
    CHECK(b1 != b2);
  }

  SUBCASE("Check operator != on identical boids") {
    BoidState b1{2., 3., 4., 0.};
    BoidState b2{2., 3., 4., 0.};
    CHECK_FALSE(b1 != b2);
  }
  SUBCASE("Check addition with two points") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{4., 3., 2., 1.};
    CHECK(((b1 + b2) ==
           {5., 5., 5., 5.}));  // mi dà errore sui check non capisco perchè
  }
  SUBCASE("Check addition with three points") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{4., 3., 2., 1.};
    BoidState b3{5., 9., 2., 6.};
    CHECK(((b1 + b2) == {10., 14., 7., 11.}));
  }
  SUBCASE("Check addition with negative values") {
    BoidState b1{-1., 2., 3., 4.};
    BoidState b2{4., -3., 2., -1.};
    CHECK((b1 + b2) == {3., -1., 5., 3.});
  }

  SUBCASE("Check addition with zero") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{0., 0., 0., 0.};
    CHECK(((b1 + b2) == {1., 2., 3., 4.}));
  }

  SUBCASE("Check difference with zero") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{0., 0., 0., 0.};
    CHECK(((b1 - b2) == {1., 2., 3., 4.}));
  }
  SUBCASE("Check difference with two points ") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{4., 3., 2., 1.};
    CHECK(((b1 - b2) == {-3., -1., 1., 3.}));
  }
  SUBCASE("Check difference with negative numbers") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{-1., -3., -2., -4.};
    CHECK(((b1 - b2) == {2., 5., 5., 8.}));
  }
  SUBCASE("Check difference which is zero") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{1., 2., 3., 4.};
    CHECK(((b1 - b2) == {0., 0., 0., 0.}));
  }
  SUBCASE("Check denominator == 0") {
    BoidState b1{2., 0., 5., 5.};
    BoidState b2{2., 3., 4., 0.};
    CHECK_FALSE((b1/b2) == 0.);
  }
  SUBCASE("Check multiplication with zero") {
    BoidState b1{2., 0., 5., 5.};
    BoidState b2{2., 3., 4., 0.};
    CHECK((b1 * b2) == {4., 0., 20., 0});
  }
}

TEST_CASE("Testing Rules"){

  SUBCASE("Check SeparationRule ")
   BoidState b1{2., 2., 5., 5.};
   BoidState b2{2., 3., 4., 0.};
   int n {2};
   double const separation_const_ {0.5};
   double const distance_s {2.};
   CHECK(v_1 == -0.5);    
   
   
   
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
 



}