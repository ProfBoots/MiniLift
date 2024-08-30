#include <Arduino.h>
#include <ESP32Servo.h>

#define RX0 3
#define TX0 1

#define pivot0 2  // Used for crane rotation
#define pivot1 4
#define boomLift0 12  // Used for lifting the boom
#define boomLift1 13

#define winch0 16  // Used for the winch which brings the hook in and out
#define winch1 17
#define extension0 25  // Used for telscoping the main boom
#define extension1 26

#define ballast0 32  // Used for controlling ballast which is the counterweight for the crane.
#define ballast1 33
#define auxAttach0 18  // used for controlling a second auxillary motor or some more lights. Keep in mind this will always breifly turn on when the model is powered on.
#define auxAttach1 5

String receivedDataStr = "";
String part1 = "";
String part2 = "";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(pivot0, OUTPUT);
  pinMode(pivot1, OUTPUT);
  pinMode(boomLift0, OUTPUT);
  pinMode(boomLift1, OUTPUT);
  pinMode(winch0, OUTPUT);
  pinMode(winch1, OUTPUT);
  pinMode(extension0, OUTPUT);
  pinMode(extension1, OUTPUT);
  pinMode(ballast0, OUTPUT);
  pinMode(ballast1, OUTPUT);
  pinMode(auxAttach0, OUTPUT);
  pinMode(auxAttach1, OUTPUT);
  digitalWrite(auxAttach0, LOW);
  digitalWrite(auxAttach1, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    receivedDataStr = Serial.readStringUntil('\n');
    int commaIndex = receivedDataStr.indexOf(',');
    Serial.print("Received: ");

    if (commaIndex != -1) {
      // Extract the first part (before the comma)
      part1 = receivedDataStr.substring(0, commaIndex);

      // Extract the second part (after the comma)
      part2 = receivedDataStr.substring(commaIndex + 1);

      // Convert part2 to an integer
      int mtr = part1.toInt();
      int velocity = part2.toInt();
      if (mtr == 1) {
        analogWrite(boomLift0, LOW);
        analogWrite(boomLift1, velocity);
      } else if (mtr == 2) {
        analogWrite(boomLift0, -1 * velocity);
        analogWrite(boomLift1, LOW);
      }
      if (mtr == 4) {
        analogWrite(pivot0, LOW);
        analogWrite(pivot1, velocity);
      } else if (mtr == 5) {
        analogWrite(pivot0, -1 * velocity);
        analogWrite(pivot1, LOW);
      }
      if (mtr == 7) {
        analogWrite(extension0, LOW);
        analogWrite(extension1, velocity);
      } else if (mtr == 8) {
        analogWrite(extension0, -1 *velocity);
        analogWrite(extension1, LOW);
      }
    }else {
      int mtr = receivedDataStr.toInt();
      if (mtr == 3) {
        analogWrite(boomLift0, LOW);
        analogWrite(boomLift1, LOW);
      }
      if (mtr == 6) {
        analogWrite(pivot0, LOW);
        analogWrite(pivot1, LOW);
      }
      if (mtr == 9) {
        analogWrite(extension0, LOW);
        analogWrite(extension1, LOW);
      }
      if (mtr == 10) {
        digitalWrite(winch0, LOW);
        digitalWrite(winch1, HIGH);
      } else if (mtr == 11) {
        digitalWrite(winch0, HIGH);
        digitalWrite(winch1, LOW);
      } else if (mtr == 12) {
        digitalWrite(winch0, LOW);
        digitalWrite(winch1, LOW);
      }
      if (mtr == 13) {
        digitalWrite(ballast0, LOW);
        digitalWrite(ballast1, HIGH);
      } else if (mtr == 14) {
        digitalWrite(ballast0, HIGH);
        digitalWrite(ballast1, LOW);
      } else if (mtr == 15) {
        digitalWrite(ballast0, LOW);
        digitalWrite(ballast1, LOW);
      }
    }
  }
}
