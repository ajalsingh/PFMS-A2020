/******************************************************************************
 * @file arrays.cpp
 * @author Ellis Tsekouras (ellis.tsekouras@uts.edu.au)
 * @brief My attempt for quiz0
 * @version 0.1
 * @date 2020-03-15
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

// ___________________________________________________________________ Includes
#include <iostream> // Includes std::cout and friends so we can output to console
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include <limits>
#include <vector>

// _____________________________________________________________________ Macros

// Create a macro (#define) to represent the max possible array size
#define ARRAY_MAX_SIZE 10

// ________________________________________________________ Function Prototypes
void    printArray(double *x, int xSize);
void    printVector(std::vector<double> &v);
void    populateWithRandomNumbers(std::vector<double> &v, int num_elements);
double  calcAvg_vector(std::vector<double> &v);
double  calcStdDev_vector(std::vector<double> &v);
void    printGTAvgStdDev(std::vector<double> &v);
void    fillVector(double x[], std::vector<double> &v);


// _______________________________________________________________________ Main

int main (void)
{
    int i;
    int num;
    std::vector<double> myVect(ARRAY_MAX_SIZE, 0);
    double x[ARRAY_MAX_SIZE] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};


/*  Rather use vectors the whole way!

    // Create new vector and fill
    for (i = 0; i < myVect.size(); i++)
    {
        myVect.at(i) = i;
    }

    printVector(myVect);     // Print the newly created vector
*/

    // Or do it their way...
    fillVector(x, myVect);
    printVector(myVect);    


    // Calculate the average and standard deviation
    std::cout << "The average is " << calcAvg_vector(myVect) << std::endl;
    std::cout << "The standard deviation is " << calcStdDev_vector(myVect) << std::endl;


    // Demonstrate dynamic length increase
    std::cout << "Increased length by 1 \n";
    myVect.push_back(6.9);

    printVector(myVect);     // Print the newly extended vector


    // Ask user to specify how many addeitional numbers are to be generated
    std::cout << "How many random elements do you wish to generate : ";

    while (!(std::cin >> num))
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input, Try again: ";
    }



    // Populate array with random numbers and print
    populateWithRandomNumbers(myVect, num);
    printVector(myVect);



     // Calculate the average and standard deviation
    std::cout << "The average is " << calcAvg_vector(myVect) << std::endl;
    std::cout << "The standard deviation is " << calcStdDev_vector(myVect) << std::endl;



    // Only print values if they're bigger than the average and standard dev.
    printGTAvgStdDev(myVect);
    

    // Main function should return an integer
    return 0;
}



// _______________________________________________________ Function Definitions


/******************************************************************************
 * @brief Function to print elements of the array
 * 
 * @param x 
 * @param xSize 
 *****************************************************************************/
void printArray(double *x, int xSize)
{
    double *ip;
    int i = 0;

    for (ip = x; ip<(x+xSize); *ip++)
    {
        std::cout << "x[" << i << "] = " << *ip << std::endl;
        i++;
    } 
}

/******************************************************************************
 * @brief Given a vector of doubles, this function will print it
 * 
 * @param num_array 
 * @param array_size 
 * @param num_elements 
 *****************************************************************************/
void printVector(std::vector<double> &v)
{
    int i = 0;
    int j = 0;

    for (i = 0; i < v.size(); i++)
    {
        std::cout << "v[" << i << "] = " << v[i] << std::endl;
    }
}


/******************************************************************************
 * @brief Function to populate array with random numbers
 * 
 * @param num_array 
 * @param array_size 
 * @param num_elements 
 *****************************************************************************/
void populateWithRandomNumbers( std::vector<double> &v, int num_elements)
{
    int i;
    unsigned int seed;

    //we select a seed for the random generator, so it is truly random (neve the same seed)
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    // create a uniform distribution between 0 and number of elements
    std::uniform_real_distribution<> value_distribution(0, num_elements);

    // generate the required amount of random numbers
    for (i = 0; i < num_elements; i++)
    {
        //num_array[i] = value_distribution(generator);
        v.push_back(value_distribution(generator));
    }
}

/******************************************************************************
 * @brief Given a vector of doubles, returns the average
 * 
 * @param v 
 * @return double 
 *****************************************************************************/
double calcAvg_vector(std::vector<double> &v)
{
    double i = 0;
    double elements = 0;
    double sum = 0;


    for (auto &i : v)
    {
        sum += i;
    }

    return (sum/v.size());
}

/******************************************************************************
 * @brief Given a vector of doubles, returns the standard deviation
 * 
 * @param v 
 * @return double 
 *****************************************************************************/
double calcStdDev_vector(std::vector<double> &v)
{
    double mu;
    double sum;

    // calculate average value
    mu = calcAvg_vector(v);

    // calculate the standard deviation
    for (auto &i : v)
    {
        sum += (i - mu)*(i - mu);
    }

    return (sqrt (sum / v.size()));
}


/******************************************************************************
 * @brief   Prints elements of the array only if they're bigger than the
 *          standard deviation and average.          
 * 
 * @param v 
 *****************************************************************************/
void printGTAvgStdDev(std::vector<double> &v)
{
    double sigma;
    double mu;
    double sigmaPlusMu;
    int i;

    // Get average and standard deviation
    sigma = calcAvg_vector(v);
    mu = calcAvg_vector(v);

    // Calculate their sum
    sigmaPlusMu = sigma + mu;
    std::cout << "Printing numbers greater than " << sigmaPlusMu << std::endl;

    // Only print the value of the vector if is big enough
    for (auto &i : v)
    {
        if (i > sigmaPlusMu)
        {
            std::cout << i << std::endl;
        }
    }

}




/******************************************************************************
 * @brief Given an array, fill up vector
 * 
 * @param x 
 * @param v 
 *****************************************************************************/
void fillVector(double x[], std::vector<double> &v)
{
    int i;

    for (i = 0; i <ARRAY_MAX_SIZE; i++)
    {
        v.at(i) = x[i];
    }
}