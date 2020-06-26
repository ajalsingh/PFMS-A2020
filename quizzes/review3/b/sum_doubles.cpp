#include <iostream>
#include <vector>
#include <random>
#include <chrono>

using std::cout;
using std::endl;
using std::vector;

// Declare the sum function ahead of main, normally we would do this in a header
// Use of & (ampersand) means we pass `numbers` by reference, avoiding a copy
// The risk of using a reference is we can change the original object
// We eliminate this risk by using the const keyword

const double minRange = 0, maxRange = 100, vecSize = 5;

double sum(const vector<double> &numbers);

int main()
{

    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> numbers(minRange, maxRange);

    std::vector<double> doubles;

    //TODO Create a vector of doubles with 4 values

    doubles.push_back(numbers(generator));
    doubles.push_back(numbers(generator));
    doubles.push_back(numbers(generator));
    doubles.push_back(numbers(generator));

    //TODO Add a value to the end/back

    doubles.insert(doubles.begin(), numbers(generator));
    //TODO Modify the 3rd value

    //TODO Print out the numbers
    // Using a Range-based for loop with the auto keyword

    for (auto it : doubles)
    {
        // std::cout << "Forloop" << std::endl;
        std::cout << "Value is: " << it << std::endl;
    }

    //TODO Compute the sum via sun function and print sum
    std::cout << "Total is: " << sum(doubles) << std::endl;
    return 0;
}

// Define the sum function, the signature must match exactly
double sum(const vector<double> &numbers)
{
    double total = 0.0;
    //TODO Use an iterator

    for (auto numIt : numbers)
    {
        total += numIt;
    }

    return total;
}
