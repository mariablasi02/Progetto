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
}

TEST_CASE("Boid already in the group") {
  BoidState b1{1., 2., 3., 4.};
  BoidState b2{2., 3., 4., 5.};
  BoidState b3{1., 4., 3., -1.};
  BoidState b{1., 2., 3., 4.};
  SeparationRule s{2., 2.};
  AlignmentRule a{0.5};
  CohesionRule c{3};
  Boids boids = {4, 3., s, a, c};
  boids.pushback(b1);
  boids.pushback(b2);
  boids.pushback(b3);
  CHECK_THROWS_AS(boids.pushback(b);, std::runtime_error);
}

TEST_CASE("Testing Separation rule") {
  SUBCASE("Testing whit a boid of the vector") {
    BoidState b1{0., 1., 2., 3.};
    BoidState b2{0., 3., 5., 1.};
    BoidState b3{2., 3., -2., 3.};
    BoidState b4{2., 1., 2., 1.};
    std::vector<BoidState> a{b1, b2, b3, b4};
    SeparationRule sr{0.7, 3};

    CHECK(sr(a, b1).val_x == doctest::Approx(-2.8));
    CHECK(sr(a, b1).val_y == doctest::Approx(-2.8));
    CHECK(sr(a, b4).val_x == doctest::Approx(2.8));
    CHECK(sr(a, b4).val_y == doctest::Approx(-2.8));
  }
  SUBCASE("Testing neighborscontrol in SeparationRule") {
    BoidState b1{0., 1., 2., 3.};
    BoidState b2{0., 3., 5., 1.};
    BoidState b3{2., 3., -2., 3.};
    BoidState b4{2., 1., 2., 1.};
    std::vector<BoidState> a{b1, b2, b3, b4};
    SeparationRule sr{0.7, 2.2};
    CHECK(sr(a, b1).val_x == doctest::Approx(-1.4));
    CHECK(sr(a, b1).val_y == doctest::Approx(-1.4));
  }
  SUBCASE("Testing a negative separation constant") {
    BoidState b1{2., 1., 2., 3.};
    BoidState b2{0., 3., 5., 1.};
    BoidState b3{2., 3., -2., 3.};
    std::vector<BoidState> v{b1, b2, b3};
    SeparationRule sr{-0.7, 3.0};
    CHECK(sr(v, b1).val_x == doctest::Approx(-1.4));
    CHECK(sr(v, b1).val_y == doctest::Approx(2.8));
    CHECK(sr(v, b2).val_x == doctest::Approx(2.8));
    CHECK(sr(v, b2).val_y == doctest::Approx(-1.4));
  }
  SUBCASE("Testing as separation constant is zero") {
    BoidState b1{4., 1., 2., 3.};
    BoidState b2{1., 3., 5., 1.};
    BoidState b3{2., 2., -2., 3.};
    std::vector<BoidState> v{b1, b2, b3};
    SeparationRule sr{0.0, 3.7};
    CHECK(sr(v, b1).val_x == doctest::Approx(0.0));
    CHECK(sr(v, b1).val_y == doctest::Approx(0.0));
    CHECK(sr(v, b2).val_x == doctest::Approx(0.0));
    CHECK(sr(v, b2).val_y == doctest::Approx(0.0));
    CHECK(sr(v, b3).val_x == doctest::Approx(0.0));
    CHECK(sr(v, b3).val_y == doctest::Approx(0.0));
  }
  SUBCASE("Boids really close") {
    BoidState n1{5., 10., 0., -1.};
    BoidState n2{5., 9.9, 0., 1.};
    std::vector<BoidState> c{n1, n2};
    SeparationRule sep{3., 10.};
    CHECK(sep(c, n1).val_y == doctest::Approx(0.3));
    CHECK(sep(c, n2).val_y == doctest::Approx(-0.3));
  }
}

TEST_CASE("Testing alignment rule") {
  SUBCASE("Boids in the same position") {
    // It is a choice to have all four boids in the same position: the formula
    // of this rule depends only on the velocity of the boids.
    BoidState b1 = {0., 0., 2., 3.};
    BoidState b2 = {0., 0., 5., 1.};
    BoidState b3 = {0., 0., -2., 3.};
    BoidState b4 = {0., 0., 1., -1};
    std::vector<BoidState> vec{b1, b2, b3, b4};
    AlignmentRule ar{0.8};
    CHECK(ar(vec, b4).val_x == doctest::Approx(0.533).epsilon(0.01));
    CHECK(ar(vec, b4).val_y == doctest::Approx(2.667).epsilon(0.01));
    BoidState b_ = {0., 0., 1.5, 1.5};
    vec.push_back(b_);
    CHECK(ar(vec, b_).val_x == 0.);
    CHECK(ar(vec, b_).val_y == 0.);
  }
  SUBCASE("Negative alignment constant") {
    BoidState b1 = {0., 0., 1., -4.};
    BoidState b2 = {0., 0., -5., 1.};
    BoidState b3 = {0., 0., -2., 3.};
    BoidState b4 = {0., 0., 1., -1};
    std::vector<BoidState> vec{b1, b2, b3, b4};
    AlignmentRule ar{-2.0};
    CHECK(ar(vec, b1).val_x == doctest::Approx(6.0).epsilon(0.01));
    CHECK(ar(vec, b1).val_y == doctest::Approx(-10.0).epsilon(0.01));
  }
  SUBCASE("Alignment constant as zero") {
    BoidState b1 = {0., 0., 1., -4.};
    BoidState b2 = {0., 0., -5., 1.};
    BoidState b3 = {0., 0., -2., 3.};
    BoidState b4 = {0., 0., 1., -1};
    std::vector<BoidState> vec{b1, b2, b3, b4};
    AlignmentRule ar{0.0};
    CHECK(ar(vec, b1).val_x == doctest::Approx(0.0).epsilon(0.01));
    CHECK(ar(vec, b1).val_y == doctest::Approx(0.0).epsilon(0.01));
  }

  SUBCASE("Alignment constant greater than 1") {
    CHECK_THROWS(AlignmentRule{1.2});
    CHECK_THROWS(AlignmentRule{5.9});
    CHECK_THROWS_AS(AlignmentRule{3.2}, std::runtime_error);
  }
}

TEST_CASE("Testing function cente_of_mass") {
  SUBCASE("Testing function centre_of_mass") {
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{2., 3., 4., 5.};
    BoidState b3{-1., 5., 6., 7.};
    std::vector<BoidState> vec{b1, b2, b3};

    CHECK(centre_of_mass(vec, b1) == Components{0.5, 4.0});
    CHECK(centre_of_mass(vec, b1).val_x == 0.5);

    CHECK(centre_of_mass(vec, b2) == Components{0.0, 3.5});
    CHECK(centre_of_mass(vec, b2).val_y == 3.5);
  }
}

TEST_CASE("Testing Cohesion rule") {
  SUBCASE("Testing with a vector of three") {
    CohesionRule c1{4.};
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{2., 3., 4., 5.};
    BoidState b3{-1., -1., -1., -1.};
    std::vector<BoidState> v1{b1, b2, b3};

    CHECK(c1(v1, b1).val_x == -2.0);
    CHECK(c1(v1, b1).val_y == -4.0);
  }

  SUBCASE("Testing with a vector of four") {
    CohesionRule c1{1};
    BoidState b1{1., 2., 3., 4.};
    BoidState b2{2., 3., 4., 5.};
    BoidState b3{-1., -1., -1., -1.};
    BoidState b4{0., -1., 3., -2.};
    std::vector<BoidState> v1{b1, b2, b3, b4};
    CHECK(c1(v1, b1).val_x == doctest::Approx(-0.67).epsilon(0.01));
    CHECK(c1(v1, b1).val_y == doctest::Approx(-1.67).epsilon(0.01));
  }

  SUBCASE("Teting with negative cohesion constant") {
    CohesionRule c1{-1.7};
    BoidState b1{1.3, 2., 3.6, 4.};
    BoidState b2{2., 3., 4., 5.};
    BoidState b3{-1., -1.3, -1., -1.7};
    BoidState b4{0., -1., 3., -2.};
    std::vector<BoidState> v1{b1, b2, b3, b4};
    CHECK(c1(v1, b1).val_x == doctest::Approx(1.64).epsilon(0.01));
    CHECK(c1(v1, b1).val_y == doctest::Approx(3.00).epsilon(0.01));
    CHECK(c1(v1, b2).val_x == doctest::Approx(3.23).epsilon(0.01));
    CHECK(c1(v1, b2).val_y == doctest::Approx(5.27).epsilon(0.01));
    CHECK(c1(v1, b3).val_x == doctest::Approx(-3.57).epsilon(0.01));
    CHECK(c1(v1, b3).val_y == doctest::Approx(-4.47661).epsilon(0.01));
    CHECK(c1(v1, b4).val_x == doctest::Approx(-1.30).epsilon(0.01));
    CHECK(c1(v1, b4).val_y == doctest::Approx(-3.79667).epsilon(0.01));
  }
  SUBCASE("Cohesion constant as zero") {
    CohesionRule c1{0.0};
    BoidState b1{1., 2., 3.5, 4.};
    BoidState b2{2.6, 3., 4., 5.4};
    BoidState b3{-1., -1., -1., -1.};
    BoidState b4{0.9, -1., 3., -2.};
    std::vector<BoidState> v1{b1, b2, b3, b4};
    CHECK(c1(v1, b1).val_x == doctest::Approx(0.0).epsilon(0.01));
    CHECK(c1(v1, b1).val_y == doctest::Approx(0.0).epsilon(0.01));
  }
}

TEST_CASE("Testing neighborcontrol function") {
  BoidState b1{1., 2., 3., 4.};
  BoidState b2{2., 3., 4., 5.};
  BoidState b3{-1., -1., -1., -1.};
  BoidState b4{0., -1., 3., -2.};
  SeparationRule s{2., 2.};
  AlignmentRule a{0.5};
  CohesionRule c{3};
  Boids pesci = {4, 3., s, a, c};
  pesci.pushback(b1);
  pesci.pushback(b2);
  pesci.pushback(b3);
  pesci.pushback(b4);
  auto n = neighborscontrol(pesci.totalboids(), b1, 3.);
  SUBCASE("Testing with a vector of four boids") {
    CHECK(static_cast<int>(n.size()) == 2);
  }
  SUBCASE("Testing with a boid on the border") {
    BoidState b5{1., 5., 0., 0.};
    pesci.pushback(b5);
    auto n = neighborscontrol(pesci.totalboids(), b1, 3.);
    CHECK(static_cast<int>(n.size()) == 3);
  }
}
TEST_CASE("Testing same_pos_check") {
  BoidState b1{2., 1., 2., 1.};
  BoidState b2{1., 4., 0.3, 1.2};
  BoidState b3{1., 2.1, 0.2, 2.};
  BoidState b4{2., 1., 2., 1.};
  std::vector<BoidState> boids{b1, b2, b3, b4};
  CHECK(same_pos_check(boids) == false);
  BoidState b5{2., 1., 2., 1.};
  BoidState b6{1., 4., 0.3, 1.2};
  BoidState b7{1., 2.1, 0.2, 2.};
  std::vector<BoidState> boid{b5, b6, b7};
  CHECK(same_pos_check(boid) == true);
}

TEST_CASE("Testing singleboid function") {
  SUBCASE("Boids in a group of three") {
    BoidState b1{0., 1., 2., 3.};
    BoidState b2{-1., 2., 3., 2.};
    BoidState b3{3., -1., 5., 2.};
    SeparationRule s{2., 4.};
    AlignmentRule a{0.5};
    CohesionRule c{4.};
    std::vector<BoidState> v1{b1, b2, b3};
    Boids b{3, 10., s, a, c};
    double const delta_t{0.1};
    CHECK(((b.singleboid(v1, b1, delta_t)).x) ==
          doctest::Approx(0.3).epsilon(0.01));
    CHECK(((b.singleboid(v1, b1, delta_t)).y) == 1.25);
    CHECK(((b.singleboid(v1, b1, delta_t)).v_x) == 3);
    CHECK(((b.singleboid(v1, b1, delta_t)).v_y) == 2.5);
  }
}

TEST_CASE("Testing evolution function") {
  SUBCASE("Testing everything without borders") {
    BoidState b1{2., 3., 4., 2.};
    BoidState b2{6., 1., -1., 1.};
    BoidState b3{4., 3., 4., 1.};
    SeparationRule s{5, 3.};
    AlignmentRule a{0.5};
    CohesionRule c{3};
    Boids bb{3, 6., s, a, c};
    bb.pushback(b1);
    bb.pushback(b2);
    bb.pushback(b3);
    CHECK(bb.totalboids().size() == 3);
    auto b_new = bb.singleboid(bb.totalboids(), b1, 0.5);
    CHECK(b_new.v_x == 1.75);
    bb.evolution(0.5);
    CHECK((bb.totalboids())[0].x == 2.875);
    CHECK((bb.totalboids())[0].y == 2.25);
    CHECK((bb.totalboids())[0].v_x == 1.75);
    CHECK((bb.totalboids())[0].v_y == -1.5);
  }

  SUBCASE("Testing velocity_limits") {
    BoidState b1{3., 1., 0.3, 2.0};
    BoidState b2{2., 1., 1.7, -0.7};
    BoidState b3{1., 4., -0.5, 0.4};
    SeparationRule s{0.2, 20};
    AlignmentRule a{0.3};
    CohesionRule c{1.3};
    Boids boid{3, 40., s, a, c};
    boid.pushback(b1);
    boid.pushback(b2);
    boid.pushback(b3);
    boid.evolution(0.3);
    CHECK((boid.totalboids())[0].v_x == doctest::Approx(-0.96).epsilon(0.01));
    CHECK((boid.totalboids())[0].v_y == doctest::Approx(2.705).epsilon(0.01));
    CHECK((boid.totalboids())[1].v_x == doctest::Approx(1.16).epsilon(0.01));
    CHECK((boid.totalboids())[1].v_y == doctest::Approx(1.22).epsilon(0.01));
  }
  SUBCASE("Testing borders") {
    BoidState b1{1179., 3., 4., 2.};
    BoidState b2{-4.3, 700., 1., 3.};
    BoidState b3{3, 700., 1., 3.};
    BoidState b4{2., -4.1, 1., 3.};
    SeparationRule s{0., 3.};
    AlignmentRule a{0.};
    CohesionRule c{0.};
    Boids bb{3, 6., s, a, c};
    bb.pushback(b1);
    bb.pushback(b2);
    bb.pushback(b3);
    bb.pushback(b4);
    CHECK(bb.totalboids().size() == 4);
    bb.evolution(0.5);
    CHECK((bb.totalboids())[0].x == 0.);
    CHECK((bb.totalboids())[1].x == 1179.);
    CHECK((bb.totalboids())[2].y == 0.);
    CHECK((bb.totalboids())[3].y == 691.);
  }
  SUBCASE("Testing with a negative value of time") {
    BoidState n1{1.507, 1.655, 2.414, 1.31};
    BoidState n2{1.7335, 0.9675, 3.427, -0.065};
    BoidState n3{2.6295, 2.3275, -4.741, -3.345};
    SeparationRule s{3, 5};
    AlignmentRule a{0.3};
    CohesionRule c{0.9};
    Boids flock{3, 300., s, a, c};
    flock.pushback(n1);
    flock.pushback(n2);
    flock.pushback(n3);
    CHECK_THROWS(flock.evolution(-0.5));
  }
}

TEST_CASE("Testing state function") {
  BoidState n1{20., 30., .4, 2.};
  BoidState n2{10., 10., 1., 1.};
  BoidState n3{40., 30., .4, 1.};
  SeparationRule s{5, 3.};
  AlignmentRule a{0.5};
  CohesionRule c{3};
  Boids bb{3, 60., s, a, c};
  bb.pushback(n1);
  bb.pushback(n2);
  bb.pushback(n3);
  auto data = statistic(bb, 0.2);
  CHECK(data.mean_distance == doctest::Approx(2.62).epsilon(0.01));
  CHECK(data.std_distance == doctest::Approx(0.39).epsilon(0.01));
  CHECK(data.mean_speed == doctest::Approx(2.36).epsilon(0.01));
  CHECK(data.std_speed == doctest::Approx(3.85).epsilon(0.01));

  BoidState b1{0., 0., 0., 0.};
  BoidState b2{0., 10., 0., 1.};
  BoidState b3{0., 30., 0., 1.};
  Boids b{3, 60., s, a, c};
  b.pushback(b1);
  b.pushback(b2);
  b.pushback(b3);
  auto value = statistic(b, 0.2);
  CHECK(value.mean_distance == doctest::Approx(2.03).epsilon(0.01));
  CHECK(value.std_distance == doctest::Approx(0.48).epsilon(0.01));
  CHECK(value.mean_speed == doctest::Approx(1.67).epsilon(0.01));
  CHECK(value.std_speed == doctest::Approx(2.72).epsilon(0.01));
}
