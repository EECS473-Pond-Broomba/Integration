#ifndef CONT_H_INC
#define CONT_H_INC

#include "stdint.h"

#define MAX_DUTY_CYCLE 750
#define MIN_DUTY_CYCLE 500
#define ROT_DUTY_CYCLE 30

#define FAR_DIST 12
#define CLOSE_DIST 2
#define WIDE_ANG 30

#define KP_Dist 5
#define KI_Dist 0.1
#define KD_Dist 1

#define KP_Bear 5
#define KI_Bear 0.1
#define KD_Bear 1

#define PID_UPDATE_TIME 1000

#define USE_PID 0 //Selecte between linear or pid. 0 = LIN, 1 = PID

class controller
{
public:
    controller();
    ~controller();

    void init();

    void setTarget(int16_t x, int16_t y);
    void update(int16_t x_curr, int16_t y_curr, uint16_t bear)
    {
    	if(USE_PID)
    	{
    		updatePidPosition(x_curr, y_curr, bear);
    	}
    	else
    	{
    		updateLinearPosition(x_curr, y_curr, bear);
    	}
    }
    void updateLinearPosition(int16_t x_curr, int16_t y_curr, uint16_t bear);
    void updatePidPosition(int16_t x_curr, int16_t y_curr, uint16_t bear);
    
    void setMotorSpeed(int pwm1, int pwm2);

    void setMotorDirection(bool left_forward, bool right_forward);

private:
    uint16_t old_distance, old_bearing;
    int16_t targetX, targetY;
    int pid_p_dist, pid_i_dist, pid_d_dist;
    int pid_p_bear, pid_i_bear, pid_d_bear;

    int16_t calculate_dist(int16_t x, int16_t y);
    int16_t calculate_bearing(int16_t x, int16_t y);

};


#endif
