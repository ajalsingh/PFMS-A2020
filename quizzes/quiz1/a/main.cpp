#include <iostream> // Includes std::cout and friends so we can output to console
#include <vector>

// Declaration of person class
#include "person.h"



//Declaration of function to greet oldest member of crowd
void oldestMember(std::vector<Person> &vec);



int main (void) {
  //Create the person 'alice'
  Person alice;
  // use member functions to set name and age
  alice.setName("Alice");
  alice.setAge(32);

  //Create the person 'bob' and print his age
  Person bob;
  bob.setName("Bob");
  bob.setAge(18);

  // Apply constraints to Bob's age
  if (bob.getAge() >= 0 && bob.getAge()< 130) {
    std::cout << bob.getName() << "'s age is " << bob.getAge() << std::endl;
  } else {
    std::cout << "Something is wrong with " << bob.getName() << "'s age" << std::endl;
  }

  //Create the person 'carol'
  Person carol;
  carol.setName("Carol");
  carol.setAge(72);

  //! TODO Create a 'crowd' using a vector container of people
  std::vector<Person> crowd;

  crowd.push_back(alice);
  crowd.push_back(bob);
  crowd.push_back(carol);

  //! TODO Create a function that greets the oldest crowd member
  oldestMember(crowd);

  //! The greeting should be: "Hello NAME, your our most distinguished member" << std::endl;
  Person mike("Mike", 22);
  std::cout<<mike.getName()<<": "<<mike.getAge()<<std::endl;
  return 0;
}



// Function to greet oldest member of crowd
void oldestMember(std::vector<Person> &vec){
    int highest_age_location = 0, highest_age = 0;
    int vec_size = vec.size();
    for (int i=0; i<vec_size; i++){
        if (vec.at(i).getAge() > highest_age){
            highest_age = vec.at(i).getAge();
            highest_age_location = i;
        }
    }
    std::cout<< "Hello "<< vec.at(highest_age_location).getName()<<", you're our most distinguished member"<< std::endl;
}
