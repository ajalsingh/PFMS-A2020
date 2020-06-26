/******************************************************************************
 * @file sample.h
 * @author Ellis Tsekouras (ellis.tsekouras@uts.edu.au)
 * @brief Header file for the class
 * @version 0.1
 * @date 2020-03-16
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

//____________________________________________________________________ Includes 
#include <iostream> // Includes std::cout and friends so we can output to console
#include <cmath>   // Includes the math library

//______________________________________________________________________ Macros 

//_______________________________________________________________________ Class

class Sample
{
//______________________________________________________________________ PUBLIC

public: 

//_________________________________________________________________ Constructor


/******************************************************************************
 * @brief Construct a new default Sample object                    
 * 
 *****************************************************************************/
Sample() : value_(0.0)
{

}



/******************************************************************************
 * @brief Construct a new user-definable Sample object            
 * 
 *****************************************************************************/
Sample(double value) : value_(value)
{
    
}

//____________________________________________________________________ Modifier

/******************************************************************************
 * @brief Set the value of the sample object                                      
 * 
 * @param value                                                       
 *****************************************************************************/
void set_value(double value)
{
    value_ = value;
}

//____________________________________________________________________ Accessor

/******************************************************************************
 * @brief   Returns the current value of the sample object             
 * 
 * @return double                                                    
 *****************************************************************************/
double get_value(void)
{
    return(value_);
}



//______________________________________________________________________ Member

// There are no unique member functions!
// It is better to keep Constructors, Modifiers and Accessors in the header file
// They typically will not change over the classes lifetime and are treated as
//  inline when implemented - making it faster

//____________________________________________________________________ Operator


/******************************************************************************
 * @brief Returns true if the value of the samples are the same, else false
 * 
 * @param sam 
 * @return true 
 * @return false 
 *****************************************************************************/
bool operator==(const Sample &sam) const;



/******************************************************************************
 * @brief Returns true if the values of the samples are indifferent, else true
 * 
 * @param sam 
 * @return true 
 * @return false 
 *****************************************************************************/
bool operator!=(const Sample &sam) const;

//_____________________________________________________________________ PRIVATE
private:

    int value_; // Value in Sample class
};
