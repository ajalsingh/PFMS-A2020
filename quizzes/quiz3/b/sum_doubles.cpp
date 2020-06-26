#include <iostream>
#include <vector>
#include <random>

using std::cout;
using std::endl;
using std::vector;

// Declare the sum function ahead of main, normally we would do this in a header
// Use of & (ampersand) means we pass `numbers` by reference, avoiding a copy
// The risk of using a reference is we can change the original object
// We eliminate this risk by using the const keyword
double sum(const vector<double> &numbers);

int main () {
    //TODO Create a vector of doubles with 4 values
    vector<double> numbers;

    std::random_device generator;
    std::uniform_real_distribution<double> distribution(0,100);
    for (int i=0;i<4;i++){
        numbers.push_back(distribution(generator));
        cout<<numbers.at(i)<<endl;
    }

    //TODO Add a value to the end/back
    cout<<"\nAdd random value to front and then replace 3rd value"<<endl;
    numbers.insert(numbers.begin(),distribution(generator));

    //TODO Modify the 3rd value
    numbers[2] = distribution(generator);

    //TODO Print out the numbers
    // Using a Range-based for loop with the auto keyword 
    for (auto number : numbers){
        cout<<number<<endl;
    }
    //TODO Compute the sum via sun function and print sum

    cout<<"\nTotal: "<<sum(numbers)<<endl;

    return 0;
}

// Define the sum function, the signature must match exactly
double sum(const vector<double> &numbers) {
    double total = 0.0;
    //TODO Use an iterator
    total = std::accumulate(numbers.begin(), numbers.end(), 0.0);
    return total;
}
