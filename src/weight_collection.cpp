/************************************
 *        weight_collection.cpp       *
 *************************************/

 /* This is for functions and tasks for
  *  finding and collecting weights  */


#include "weight_collection.h"
#include "Arduino.h"
#include "BigServo.h"



void weight_scan(/* whatever parameters */) 
{
  /* Use sensors to search for weights,
   * Switch to WEIGHT_FOUND state if a weight is found   */
   Serial.println("Looking for weights \n");
}


void collect_weight(int collect_time)
{
  /* When ready, collect the weight */
  Serial.println("Collecting weight \n");
  //digitalWrite(ELECTROMAGNET_PIN, LOW);
  if (collect_time < 15) {
    digitalWrite(ELECTROMAGNET_PIN, HIGH);
    int startangle = rest_angle + 1 + ((PICKUP_ANGLE - rest_angle) * collect_time) / 15;
    bigServo_move(startangle);
  } else if (collect_time < 18) {
    bigServo_move(PICKUP_ANGLE);
  } else if (collect_time < 55) {
    int midangle = PICKUP_ANGLE - ((PICKUP_ANGLE - DROPOFF_ANGLE) * (collect_time - 15)) / 40;
    bigServo_move(midangle);
  } else  if (collect_time < 65){
    bigServo_move(DROPOFF_ANGLE);
    digitalWrite(ELECTROMAGNET_PIN, LOW);
  } else {
    bigServo_move(rest_angle);
  }
}

