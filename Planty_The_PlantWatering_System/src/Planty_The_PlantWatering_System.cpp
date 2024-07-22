/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);

int Last500MillisSec;
int currentTime;

// Run the application and system concurrently in separate threads
//SYSTEM_THREAD(ENABLED);

// setup() runs once, when the device is first turned on
void setup() {
  pinMode(S4,OUTPUT);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  if((currentTime -Last500MillisSec)>500) {
    Last500MillisSec = millis();
   digitalWrite(S4,LOW);
  }
  else{digitalWrite(S4,HIGH);}
  }


