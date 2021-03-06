#include <string>
#include <sstream> // For easily converting numbers to strings
#include <math.h>
#include "complex.h"

TComplex Complex::magnitude(TComplex a) {
  return {sqrt(pow(a.re, 2) + pow(a.im, 2))};
}

TComplex Complex::conjugate(TComplex a) {
  return {a.re, -a.im};
}

TComplex Complex::add(TComplex a, TComplex b) {
  return {(a.re + b.re), (a.im + b.im)};
}

TComplex Complex::subtract(TComplex a, TComplex b) {
  return {(a.re - b.re), (a.im - b.im)};
}

TComplex Complex::multiply(TComplex a, TComplex b) {
    return {((a.re * b.re) + (a.im * b.im)), ((a.re * b.im) + (a.im * b.re))};
}

TComplex Complex::divide(TComplex a, TComplex b) {
  return {((a.im / b.re) - (b.im / a.re)), ((a.im / a.re) - (b.im * b.re))};
}

std::string Complex::format(TComplex a) {
  std::stringstream ss;
  ss << a.re << " + " << a.im << "i";
  return ss.str();
}
