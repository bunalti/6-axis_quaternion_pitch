 /* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
float angle;
unsigned long time_start;
long pos;
float velocity;
double delta;

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(2, 3);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(115200);
  Serial.println("Basic Encoder Test:");

  time_start = millis();
}

long oldPosition  = 0;

void loop() {
  long newPosition = myEnc.read();
  delta = (1.0f/60.0f) * 0.001f * (millis()-time_start);

  velocity = ((newPosition-oldPosition)/3591.84f) / delta;
  
  pos = newPosition % 3592;
  angle = (pos/3591.84f) * 360.0f;
  

  
  Serial.print(velocity,2);
  Serial.print(",\t");
  Serial.println(angle);


  oldPosition = newPosition;
  time_start = millis();
  delay(1);

  
}
