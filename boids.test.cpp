#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boids.hpp"
#include "doctest.h"

TEST_CASE("Testing norm function") {
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

TEST_CASE("Testing Separation rule") {
  SUBCASE("General test ") {
    BoidState b1 = {0., 1., 2., 3.};
    BoidState b2 = {0., 3., 5., 1.};
    BoidState b3 = {2., 3., -2., 3.};
    std::vector<BoidState> a{b1, b2, b3};
    BoidState b = {2., 3., 1., 2.};
    SeparationRule sr{4, 0.5, 6.};

    CHECK(sr(a, b).vel_x == doctest::Approx(-2.));
    CHECK(sr(a, b).vel_y == doctest::Approx(-1.));

    BoidState b_ = {0., 0., 0., 0.};

    CHECK(sr(a, b_).vel_x == doctest::Approx(1.));
    CHECK(sr(a, b_).vel_y == doctest::Approx(3.5));
  }
};
/*
TEST_CASE("Testing alignment rule") {
  SUBCASE("General tests") {
    BoidState b1 = {0., 0., 2., 3.};
    BoidState b2 = {0., 0., 5., 1.};
    BoidState b3 = {0., 0., -2., 3.};
    std::vector<BoidState> a{b1, b2, b3};
    BoidState b = {0., 0., 1., -1};
    AllignmentRule ar{4, 0.8};
    CHECK(ar(a, b).vel_x == doctest::Approx(1.064).epsilon(0.01));
    CHECK(ar(a, b).vel_y == doctest::Approx(2.136).epsilon(0.01));
    BoidState b_ = {0., 0., 5., 7.};
    CHECK(ar(a, b_).vel_x == 0.);
    CHECK(ar(a, b_).vel_y == 0.);
  }
  SUBCASE("a greater than 1") { CHECK_THROWS(AllignmentRule{5, 1.2}); }
  /*SUBCASE("Trying to break the code"){
    //non ho idee per ora
  }*/


TEST_CASE("Testing Cohesion rule") {
  SUBCASE("testing with a vector of three") {
    CohesionRule c1{3, 4};
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{2., 3., 4., 5.};
    BoidState b3{-1., -1., -1., -1.};
    std::vector<BoidState> v1{b1, b2, b3};

    CHECK(c1(v1).vel_x == -2.0);
    CHECK(c1(v1).vel_y == -4.0);
  }

  SUBCASE("testing with a vector of four") {
    CohesionRule c1{4, 1};
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{2., 3., 4., 5.};
    BoidState b3{-1., -1., -1., -1.};
    BoidState b4{0., -1., 3., -2.};
    std::vector<BoidState> v1{b1, b2, b3, b4};
    CHECK(c1(v1).vel_x == doctest::Approx(-0.67).epsilon(0.01));
    CHECK(c1(v1).vel_y == doctest::Approx(-1.67).epsilon(0.01));
  }

  /* SUBCASE("try single functions") {
    // CohesionRule c1{3, 4};
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{2., 3., 4., 5.};
    BoidState b3{-1., -1., -1., -1.};
    std::vector<BoidState> v1{b1, b2, b3};
    int n = 3;
    CHECK(COM(n, v1).vel_y == 0.5);
    // CHECK(c1(v1).vel_y == 2.);
  }*/
}

TEST_CASE("Testing Neighbor-Control function") {
  BoidState b1{1., 2., 3., 4.};
  BoidState b2{2., 3., 4., 5.};
  BoidState b3{-1., -1., -1., -1.};
  BoidState b4{0., -1., 3., -2.};
  SeparationRule s{3, 2., 2.};
  AllignmentRule a{3, 0.5};
  CohesionRule c{3, 3};
  Boids pesci = {4, 3., s, a, c};
  pesci.push_back(b1);
  pesci.push_back(b2);
  pesci.push_back(b3);
  pesci.push_back(b4);
  auto b = NeighborsControl(pesci.TotalBoids(), b1, 3.);
  CHECK(static_cast<int>(b.size()) == 2);
}


TEST_CASE("Testing singleboid function") {
  SUBCASE("boid in a group of three") {
    BoidState b1{0., 1., 2., 3.};
    BoidState b2{-1., 2., 3., 2.};
    BoidState b3{3., -1., 5., 2.};
    SeparationRule s{3, 2., 4.};
    AllignmentRule a{3, 0.5};
    CohesionRule c{3, 4.};
    std::vector<BoidState> v1{b1, b2, b3};
    Boids b{3, 10., s, a, c};
    double const delta_t{0.1};
    /*CHECK(((b.singleboid(v1, b1, delta_t)).x) == 1.2);
    CHECK(((b.singleboid(v1, b1, delta_t)).y) == 1.);
    CHECK(((b.singleboid(v1, b1, delta_t)).v_x) == 12.);
    CHECK(((b.singleboid(v1, b1, delta_t)).v_y) == 0.);*/
    //CHECK((b.singleboid(b1, delta_t).x) == (0.8));
    //CHECK((b.singleboid(b1, s(v1, b1), a(b1, v1), c(v1), delta_t).y) == 
    // (1.05)); CHECK((b.singleboid(b1, s(v1, b1), a(b1, v1), c(v1),
    // delta_t).v_x) == (8)); CHECK((b.singleboid(b1, s(v1, b1), a(b1, v1),
    // c(v1), delta_t).v_y) ==
    //     (0.5));
  }
}

TEST_CASE("Testing Boids with the same position") {
  SUBCASE("Testing Boids") {
    BoidState b1{7., 2., 2., 3.};
    BoidState b2{3., 2., 3., 4.};
    BoidState b3{6., 2., 3., 3.};
    std::vector<BoidState> a{b1, b2, b3};
    BoidState b{1., 2., 2., 3.};
    same_position(b, a);
    CHECK(static_cast<int>(a.size()) == 3);
  }
}

