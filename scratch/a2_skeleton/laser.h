/**
 * @file    laser.h
 * @ingroup Ranger
 * @class   Laser
 * @brief   The Laser class is a child class the Ranger Class.
 *          \n This class creates lasers of sensing method type POINT
 * @author  Ajal Singh
 * @version 1.0
 * @date    April 2020
 */

#ifndef LASER_H
#define LASER_H

#include "ranger.h"

//_____________________________________________________________________________ Class Definition

class Laser: public Ranger
{

//_____________________________________________________________________________ Class Public Members

public:
  //Default constructor - should set all sensor attributes to a default value
  /**
   * @brief Laser default constructor
   */
  Laser();

  /**
   * @brief Laser default destructor
   */
  ~Laser();

};

#endif // LASER_H
