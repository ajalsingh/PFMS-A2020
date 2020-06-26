#include "person.h"
#include <iostream> // Includes std::cout and friends so we can output to console


// Default constructor
Person::Person(): Person(NULL, 0){}

// Constructor with arguments
Person::Person(std::string name, int age): name_(name), age_(age){}

void Person::setName(std::string name) {
  name_ = name;
}

bool Person::setAge(int age) {
  if (age >= 0 && age< 130){
      age_ = age;
  }
  else{
      age_ = 0;
  }
  return true;
}

std::string Person::getName(void) {
  return name_;
}

int Person::getAge(void) {
  return age_;
}
