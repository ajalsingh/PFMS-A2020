#include "laser.h"
/**
 * @brief   Default constructor sets the fixed params.
 *          \n  setModel:               "SICK-XL"
            \n setFieldOfView:         180 degrees
            \n setSensingMethod:       POINT
            \n setAngularResolution:   10  degrees
            \n setMaxRange:            8   metres
            \n setMinRange:            0.2 metres
 */
Laser::Laser(){
    setModel("SICK-XL");
    setFieldOfView(180);        // Field of view 180 degrees
    setSensingMethod(POINT);
    setAngularResolution(10);   // Default angular resolution 10 degrees
    setMaxRange(8.0);           // Max range set to 8.0 metres
    setMinRange(0.2);           // Min range set to 1.0 metre
}

Laser::~Laser(){}

