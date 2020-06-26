/**
 * @file    sonar.h
 * @ingroup Ranger
 * @class   Sonar
 * @brief   The Sonar class is a child class the Ranger Class
 *          Creates sonar objects of sensing method type CONE
 * @author  Ajal Singh
 * @version 1.0
 * @date    April 2020
 */

#ifndef SONAR_H
#define SONAR_H

#include "ranger.h"

//_____________________________________________________________________________ Class Definition

class Sonar: public Ranger
{
//_____________________________________________________________________________ Class Public Members

public:
    //Default constructor should set all sensor attributes to a default value
    /**
     * @brief Sonar default constructor
     */
    Sonar();

    /**
     * @brief Sonar default destructor
     */
    ~Sonar();

};

#endif // SONAR_H
