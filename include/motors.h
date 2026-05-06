//************************************
//         motors.h    
//************************************

#ifndef MOTORS_H_
#define MOTORS_H_

// SET THIS TO REAL VALUES
#define LEFT_MOTOR_ADDRESS 1      //Pin corresponding to the left dc motor
#define RIGHT_MOTOR_ADDRESS 1     //Pin corresponding to the right dc motor
#define MIN_SPEED_CAP 1           //Set the minimum speed value that can be written to the motors
#define MAX_SPEED_CAP 1           //Set the maximum speed value that can be written to the motors

void check_speed_limits(/*parameters*/);
void set_motor(/*parameters*/);


#endif /* MOTORS_H_ */
