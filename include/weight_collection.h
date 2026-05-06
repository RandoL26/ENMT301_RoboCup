
/************************************
 *        weight_collection.h       *
 *************************************/

 /* This header is for functions and tasks for
  *  finding and collecting weights  */

#ifndef WEIGHT_COLLECTION_H_
#define WEIGHT_COLLECTION_H_

#include <Servo.h>                  //control the DC motors
//#include <Herkulex.h>               //smart servo
#include <Wire.h>                   //for I2C and SPI


#include "sensors.h"               //will need sensor library to detect weights


//states for swapping between searching and collecting
#define NO_WEIGHT               0   
#define WEIGHT_FOUND            1



void weight_scan(/* whatever parameters */);


void collect_weight();



#endif /* WEIGHT_COLLECTION_H_ */
