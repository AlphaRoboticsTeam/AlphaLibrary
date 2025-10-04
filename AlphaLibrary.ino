#include "AlphaTeam.h"
AlphaMotor motorB(2,3,5);
AlphaColorSensor cs(3,HSV);
AlphaDistanceSensor ultra(8,9);
AlphaGyroSensor gyro(2);
AlphaBoard board(13);

void setup() {
  // put your setup code here, to run once:
board.waitFor();
cs.begin();
gyro.begin();
ultra.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}
