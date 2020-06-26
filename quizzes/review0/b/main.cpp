/******************************************************************************
 * @file main.cpp
 * @author Ellis Tsekouras (ellis.tsekouras@uts.edu.au)
 * @brief main for quiz0b
 * @version 0.1
 * @date 2020-03-16
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/


//____________________________________________________________________ Includes 
// We need to include the declaration of our new rectangle class in order to use it.
//#include "rectangle.h"
#include <iostream>
#include "sample.h"

//______________________________________________________________________ Macros 

//_____________________________________________________________________ Globals

//_________________________________________________________ Function Prototypes

//________________________________________________________________________ Main
int main (void)
{
/*
    // Create a rectangle object
    Rectangle rectangle;

    // Set the values of the sides
    rectangle.setWidthHeight(5.0,5.0);

    // Get the area and print it to screen
    double result = rectangle.area();
    std::cout << result << std::endl;
*/
    // Create a new sample object and set value to 10
    Sample sample0;
    Sample sample1(10);


    std::cout << "Sample0 is " << sample0.get_value() << std::endl;
    std::cout << "Sample1 is " << sample1.get_value() << std::endl;

    sample0.set_value(sample0.get_value()+1);

    std::cout << "Sample is " << sample0.get_value() << std::endl;



    if (sample0 != sample1)
    {
        std::cout << "Samples are different" << std::endl;
    }


    // Set the values to be the same
    sample0.set_value(sample1.get_value());

    //std::cout << "Sample0 is " << sample0.get_value() << std::endl;
    //std::cout << "Sample1 is " << sample1.get_value() << std::endl;

    if (sample0 == sample1)
    {
        std::cout << "Samples are the same" << std::endl;
    }

    return 0;
}
