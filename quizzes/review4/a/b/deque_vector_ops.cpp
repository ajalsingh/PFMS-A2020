#include <iostream>
#include <deque>
#include <chrono>
#include <random>
#include <vector>

#define MEAN 8.0
#define STD_DEV 4.0

//! Sample function for the creating elements of the deque
std::deque<double> populateDeque(std::deque<double>& values, int num_values, double mean, double std_dev)
{
	// Create a random number generator and seed it from
	// the system clock so numbers are different each time
    //http://www.cplusplus.com/reference/random/normal_distribution/normal_distirbution/
    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<double> distribution (mean,std_dev);

    for (int i=0; i<num_values; ++i)
    {
        values.push_back(distribution(generator));
    }
    return values;
}

//! Sample function for printing elements of the deque
void printDeque(std::deque<double>& values)
{
    std::cout << "Deque:-" << std::endl;
	// Loop through the deque and print the values
    for (auto value : values)
    {
		std::cout << value << " ";
	}
	std::cout << std::endl;
}

std::vector<double> populateVector(std::vector<double>& vecs, int num_values, double mean, double std_dev)
{
    // Create a random number generator and seed it from
    // the system clock so numbers are different each time
    //http://www.cplusplus.com/reference/random/normal_distribution/normal_distirbution/
    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<double> distribution (mean,std_dev);

    for (int i=0; i<num_values; ++i)
    {
        vecs.push_back(distribution(generator));
    }
    return vecs;
}

void printVector(std::vector<double>& vecs)
{
    std::cout << "Vector:-" << std::endl;
    // Loop through the vector and print the values
    for (auto value : vecs)
    {
        std::cout << value << " ";
    }
    std::cout << std::endl;
    std::cout << std::endl;
}

std::deque<double> sortDeque(std::deque<double>& values, int num_values)
{
    int i, j, pass=0;
    double temp;

    // Loop through the deque and sort the values
    for(i = 0; i<num_values; i++)
    {
       for(j = i+1; j<num_values; j++)
       {
          if(values[j] < values[i])
          {
             temp = values[i];
             values[i] = values[j];
             values[j] = temp;
          }
       }
    pass++;
    }
    return values;
}

std::vector<double> sortVector(std::vector<double>& vecs, int num_values)
{
    int i, j, pass=0;
    double temp;

    // Loop through the vector and sort the values
    for(i = 0; i<num_values; i++)
    {
       for(j = i+1; j<num_values; j++)
       {
          if(vecs[j] < vecs[i])
          {
             temp = vecs[i];
             vecs[i] = vecs[j];
             vecs[j] = temp;
          }
       }
    pass++;
    }
    return vecs;
}

int main()
{
    ////////////////////////////////////////////////

	// Create an empty deque
	std::deque<double> values;
	// Populate it with random numbers
    int num_values;
    std::cout << "Enter Number of Elements:-" << std::endl;
    std::cin >> num_values;
    populateDeque(values, num_values, MEAN, STD_DEV);

	// Print the contents of the deque
    std::cout << "UnSorted Elements" << std::endl;
    printDeque(values);

    ////////////////////////////////////////////////

	// Create an empty vector
    std::vector<double> vecs;

	// Populate it with random numbers
    populateVector(vecs, num_values, MEAN, STD_DEV);

	// Print the contents of the vector
    printVector(vecs);
    ////////////////////////////////////////////////

    // Bubble sort one of your containers
    sortDeque(values, num_values);
    sortVector(vecs, num_values);

	// Print the contents of the container
    std::cout << "Sorted Elements" << std::endl;
    printDeque(values);
    printVector(vecs);

	return 0;
} 
