// We need to include the declaration of our new rectangle class in order to use it.
#include "sample.h"

#include <iostream>

int main () {

    // Create a sample object
    Sample sample(5.5);
    std::cout<< "Value is: "<< sample.readvalue()<<std::endl;

    sample.setvalue(2.2);
    std::cout<< "Value is: "<< sample.readvalue()<<std::endl;

    return 0;
}
