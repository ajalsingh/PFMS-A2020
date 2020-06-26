#include <iostream>
#include <deque>
#include <chrono>
#include <random>
#include <vector>

void populateDeque(std::deque<double>& values, int num_values, double mean, double std_dev);
void printDeque(std::deque<double>& values);
void bubbleSort(std::deque<double>& values);
void printVec(std::vector<double>& values);
void populateVec(std::vector<double>& values, int num_values, double mean, double std_dev);
void bubbleSortVec(std::vector<double>& values);

int main() {

    ////////////////////////////////////////////////

	// Create an empty deque
	std::deque<double> values;
	// Populate it with random numbers
    const int mean = 8;
    const int std_dev = 4;
    int input;

    std::cout<<"How many elements would you like to add?"<<std::endl;
    std::cin>>input;
    populateDeque(values, input, mean, std_dev);
    std::cout<<"\nDeque:"<<std::endl;

	// Print the contents of the deque
    printDeque(values);


    ////////////////////////////////////////////////


    // Bubble sort one of your containers
    bubbleSort(values);
	// Print the contents of the container
    printDeque(values);

    ////////////////////////////////////////////////

    // Create an empty vector
    std::vector<double> vec;
    // Populate it with random numbers
    populateVec(vec, input, mean, std_dev);
    std::cout<<"\nVector:"<<std::endl;

    // Print the contents of the vector
    printVec(vec);

    // Vector sort
    bubbleSortVec(vec);
    printVec(vec);
	return 0;
} 


//! Sample function for the creating elements of the deque
void populateDeque(std::deque<double>& values, int num_values, double mean, double std_dev) {

    // Create a random number generator and seed it from
    // the system clock so numbers are different each time
    int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(seed);
    std::normal_distribution<double> distribution(mean,std_dev);

    //http://www.cplusplus.com/reference/random/normal_distribution/normal_distirbution/

    for (int i=0; i<num_values; i++){
        values.push_back(distribution(gen));
    }
}

//! Sample function for printing elements of the deque
void printDeque(std::deque<double>& values) {
    // Loop through the deque and print the values
    for (auto value : values) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
}

void bubbleSort(std::deque<double>& values){
    bool complete = false;
    int no_swap;

    while (complete == false){
        no_swap = 0;
        for (int i=0;i<values.size()-1;i++){
            if (values[i] > values[i+1]){
                std::swap(values[i],values[i+1]);

                //uncomment to see each pass
                //printDeque(values);
            }
            else{
                no_swap++;
            }
        }
        if (no_swap == values.size()-1){
            complete = true;
        }
    }
}

void printVec(std::vector<double>& values) {
    for (auto value : values) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
}

void populateVec(std::vector<double>& values, int num_values, double mean, double std_dev) {

    // Create a random number generator and seed it from
    // the system clock so numbers are different each time
    int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(seed);
    std::normal_distribution<double> distribution(mean,std_dev);

    //http://www.cplusplus.com/reference/random/normal_distribution/normal_distirbution/

    for (int i=0; i<num_values; i++){
        values.push_back(distribution(gen));
    }
}

void bubbleSortVec(std::vector<double>& values){
    bool complete = false;
    int no_swap;

    while (complete == false){
        no_swap = 0;
        for (int i=0;i<values.size()-1;i++){
            if (values[i] > values[i+1]){
                std::swap(values[i],values[i+1]);

                //uncomment to see each pass
                //printVec(values);
            }
            else{
                no_swap++;
            }
        }
        if (no_swap == values.size()-1){
            complete = true;
        }
    }
}
