#include "gtest/gtest.h"
#include <iostream>
#include <cmath>
#include "complex.h"

TComplex a = { .re = 2, .im = 3 };
TComplex b = { 1, 1 };

//The 'TEST' keyword signals to the gtest framework that this is a unit test
TEST (ComplexNumTest, Divide) {
  //Two different methods of initialising a struct
  // TComplex a = { .re = 2, .im = 3 };
  // TComplex b = { 1, 1 };
  TComplex expected = { 2.5, 0.5 };
  TComplex answer = Complex::divide(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST (ComplexNumTest, add){
  TComplex expected = { 3, 4 };
  TComplex answer = Complex::add(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST (ComplexNumTest, subtract){
  TComplex expected = { 1, 2};
  TComplex answer = Complex::subtract(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST (ComplexNumTest, multiply){
  TComplex expected = {-1, 5};
  TComplex answer = Complex::multiply(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST (ComplexNumTest, magnitude){
  double expected = {sqrt(13)};
  double answer = Complex::magnitude(a);

  //Check that the expected is equal to the actual answer
  EXPECT_DOUBLE_EQ(expected, answer);
}

TEST (ComplexNumTest, conjugate){
  TComplex expected = {a.re, -3};
  TComplex answer = Complex::conjugate(a);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST (ComplexNumTest, format){
  std::string expected = "2 + 3i";
  std::string answer = Complex::format(a);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected, answer);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
