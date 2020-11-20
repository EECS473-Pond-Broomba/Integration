#ifndef CONT_H_INC
#define CONT_H_INC

#include "stdint.h"

#define MAX_DUTY_CYCLE 750
#define MIN_DUTY_CYCLE 600
#define ROT_DUTY_CYCLE 30

#define FAR_DIST 12
#define CLOSE_DIST 2
#define WIDE_ANG 30

class controller
{
public:
    controller();
    ~controller();

    void init();

    void setTarget(int16_t x, int16_t y);
    void updatePosition(int16_t x_curr, int16_t y_curr, uint16_t bear);
    
private:
    uint16_t old_distance, old_bearing;
    int16_t targetX, targetY;

    int16_t calculate_dist(int16_t x, int16_t y);
    uint16_t calculate_bearing(int16_t x, int16_t y);

};


#endif
