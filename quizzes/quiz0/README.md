Quiz 0
======

Part A
------
1) Attempt to compile the code in [folder a](./a). Why does the code fail to compile?
Code fails to compile as there are 2 functions of the same name. Since both functions are intended to do the same thing we only need 1.

2) Fix the compilation error. Why does a segmentation fault (segfault) occur?
They occur when trying to read or write an illegal memory location. When a reference to a variable falls outside the segment in which the variable belongs. The array in this example has been initialised as 10 bits which is too small when we are trying to add elements to the array.

3) Modify the code so it will run
Increase the array size to accept more than 10 elements.

4) Create a function that prints only the elements of the array that are larger than : mean + one standard deviation

5) Create a function that assigns elements of array x to a vector named `vec` (HINT: decide on the correct type)

Part B
------
1) Look at the code in [folder b](./b). Implement the methods of the Sample class in the [sample.cpp](./b/sample.cpp) file based on the definition provided in [sample.h](./b/sample.h)

2) Make an executable that creates an object `sample` of `Sample` class and then obtains the value of parameter `value_` in this object.

3) What do we call functions of a class?
Function Members, sometimes called methods.

4) What access specifiers are used?
Private and Public. Private is used to hold data members while public is used to hold the interface or function members/methods.

5) What do we call variable `value_` in the `Sample` class?
Private data member.
