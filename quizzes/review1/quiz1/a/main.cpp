#include "person.h"
#include <iostream> // Includes std::cout and friends so we can output to console
#include <vector>

void greetOldest(std::vector<Person> group) {
  Person oldestPerson ("Unknown", 0);
  for (int i=0;i<group.size();i++) {
    if (group[i].getAge()>oldestPerson.getAge()) {
      oldestPerson = group[i];
    }
  }
  std::cout << "Hello " << oldestPerson.getName() << ", you're our most distinguished member" << std::endl;
}

int main (void) {
  //Create the person 'alice'
  Person alice("Alice", 32);
  // alice.setName("Alice");
  // alice.setAge(32);

  //Create the person 'bob' and print his age
  Person bob("Bob", -62);
  // bob.setName("Bob");

  if (bob.setAge(-62)) {
    std::cout << bob.getName() << "'s age is " << bob.getAge() << std::endl;
  } else {
    std::cout << "Something is wrong with " << bob.getName() << "'s age" << std::endl;
  }

  //Create the person 'carol'
  Person carol("Carol", 72);
  // carol.setName("Carol");
  // carol.setAge(72);

  //! TODO Create a 'crowd' using a vector container of people
  std::vector<Person> crowd;
  crowd.push_back(alice);
  crowd.push_back(bob);
  crowd.push_back(carol);

  std::cout << "There are " << crowd.size() << " people in the crowd" << std::endl;

  //! TODO Create a function that greets the oldest crowd member
  //!
  //! The greeting should be: "Hello NAME, you're our most distinguished member" << std::endl;

  greetOldest(crowd);

  return 0;
}
