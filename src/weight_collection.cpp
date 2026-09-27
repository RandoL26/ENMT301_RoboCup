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


void collect_weight()
{
  /* When ready, collect the weight */
  Serial.println("Collecting weight \n");
  digitalWrite(ELECTROMAGNET_PIN, LOW);
}

