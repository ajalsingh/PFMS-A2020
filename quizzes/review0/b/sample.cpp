/******************************************************************************
 * @file sample.cpp
 * @author Ellis Tsekouras (ellis.tsekouras@uts.edu.au)
 * @brief Source file for the Sample Class
 * @version 0.1
 * @date 2020-03-16
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

//____________________________________________________________________ Includes 
#include "sample.h"

//______________________________________________________________________ Macros 

//_____________________________________________________________________ Globals

//_________________________________________________________ Function Prototypes

//__________________________________________________ Member Function Defintions

//________________________________________________________ Operator Definitions

/******************************************************************************
 * @brief Returns true if the value of the samples are the same, else false
 * 
 * @param sam asdas
 * @return true 
 * @return false 
 *****************************************************************************/
bool Sample::operator==(const Sample &sam) const
{
    return(value_ == sam.value_);
}



/******************************************************************************
 * @brief Returns true if the values of the samples are indifferent, else true
 * 
 * @param sam   Other object to be comapred against
 * @return true 
 * @return false 
 *****************************************************************************/
bool Sample::operator!=(const Sample &sam) const
{
        return(value_ != sam.value_);
}


//_________________________________________________________ Function Defintions

