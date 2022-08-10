

SUBCASE ("norm2 <0" ){
    BoidState b1{2.,3.,5.,5.};
    BoidState b2{2.,3.,4.,4.};
    double norm2 = (b1.x - b2.x) * (b1.x - b2.x) + (b1.y - b2.y) * (b1.y - b2.y);
    CHECK(norm2(b1,b2)==doctest::Approx(0.));
}


SUBCASE ("Denominator == 0"){
    BoidState b1{2.,0.,5.,5.};
    BoidState b2{2.,3.,4.,0.}; 
    CHECK(operator/(b1,b2));
}