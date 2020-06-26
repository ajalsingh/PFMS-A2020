Quiz 4
======

Part A
------

1) TASK: Create a function that accepts a deque and modifies it by adding user specified numbers of elements, each element is form a gaussian distribution (mean: 8, std dev 4) [deque_vector_ops.cpp](./a/deque_vector_ops.h)

2) TASK: Create a functions that prints the chosen container

3) We are now tasked to design a function to rearrange elements of out container (vector or deque) by bubble sort operation,refer for pseudo code  (https://en.wikipedia.org/wiki/Bubble_sort).
QUESTION: Which STL container is more suitable for the Bubble sort and why?
    - Bubble sort should be avoided for large data sets. We know vectors are the preferred container for large data sets, hence a deque will ensure the data set is small. Deque is more suitable.

4) TASK: Create a function that accept the chosen container and rearranges elements by bubble sort operation

5) TASK: In the main call sorting function and print the container after operation (re-use function developed in step 2)

Part C
-------

Consider the code in [data_race.cpp](./b/data_race.cpp)  

1) QUESTION: How many threads are running in parallel (disregarding the main)?
    - 2

2) QUESTION: What value would you expect to see printed?
    - 20000000

3) QUESTION: What is the specific problem with this code, causing it to fail?
    - The problem is race condition where both threads are 'racing' for common recources. This led the output to fluctuate and not reach the intended value. :shit:

4) TASK: Implement one approach to fix the problem and outline merits of the solution.
    - Mutex is used to solve this problem. This synchronises the access of common resources by threads.

5) TASK: Instead of having `int shared_int` as a global variable, change the code to pass this variable to function `increment ()`


