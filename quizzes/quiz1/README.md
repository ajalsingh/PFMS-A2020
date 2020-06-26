Quiz 1
======

Part A
------

1) QUESTION Attempt to compile the code in [folder a](./a). Why does it fail? (list the reasons)
- The person class is not declared in the main.cpp file (#include "person.h".
- Cannot write directly into the member variables (name_, age_). We must use the member functions to set name and age. 

2) TASK Fix the issue so that it compiles.

3) TASK Make the code more robust, with respect to "sane" values of age.

4) TASK Create a `crowrd` using a vector container of people, populate it with 3 people.

5) TASK Create a function that greets the oldest member of the `crowd`.

6) TASK Implement a safe guard to make the initialisation of `person` objects easier? (HINT: What special member function is missing in Person?)

Part B
------

1) TASK Modify the file rectangle [rectangle](./b/rectangle.h) so it inherits from the base class of shape is [shape](./b/shape.h)

2) TASK Correct the missing access specifiers of base class [shape](./b/shape.h)

3) TASK Modify the main [main.cpp](./b/main.cpp) so that it creates a rectangle of size width=5 and height =3.5

4) QUESTION If you create a `Rectangle`, which constructor is called, and what does it initialise?
When Rectangle is created, first an instance of the Shape class is created. This initialises the 'description_' variable to "unknown shape". Then an instance of the Rectangle class is created which assigns a value of 0 to both the heigh and width parameters.
