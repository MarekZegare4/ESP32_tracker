#include <Arduino.h>
#include <SCServo.h>

void trackingTask(void * parameters);
void trackingInitialize();
float getCompassDegree();
int readCurrent();
void servoDemo();