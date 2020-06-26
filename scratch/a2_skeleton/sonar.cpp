#include "sonar.h"
/**
 * @brief   Default constructor sets the parameters to fixed params.
 *          Sets model, field of view, sensing method, angular resolution, max range and min range.
 */
Sonar::Sonar(){
    setModel("SN-001");
    setSensingMethod(CONE);
    setFieldOfView(20);                             // Field of view 20 degrees
    setAngularResolution(this->getFieldOfView());   // Angular resolution set to be the same as field of view but is not required for this sensor
    setMaxRange(16.0);                              // Max range set to 16.0 metres
    setMinRange(0.2);                               // Min range set to 0.2 metres
}

Sonar::~Sonar(){}
