#include "person.h"
#include <iostream> // Includes std::cout and friends so we can output to console

void Person::setName(std::string name) {
  name_ = name;
}

bool Person::setAge(int age) {
  if (age > 0 && age < 125) {
    age_ = age;
    return true;
  }
  else {
    return false;
  }
}

std::string Person::getName(void) {
  return name_;
}

int Person::getAge(void) {
  return age_;
}