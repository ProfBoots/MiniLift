#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>
//25,26,32,33,21,19,22,23,2,4,17,16

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

/*Serial commands for crane esp32 daughter board
1-Boom up
2-Boom down
3-Boom stop
4-Pivot Left
5-Pivot Right
6-Pivot stop
7-Extension out
8-Extension in
9-Extension stop
10-Winch out
11-Winch in
12-Winch stop
13-Ballast up
14-Ballast down
15-Ballast stop
*/
#define cabLights 32
#define auxLights 33

#define frontSteeringServoPin 23
#define rearSteeringServoPin 22
#define frontMotor0 32  // Used for controlling front drive motor movement
#define frontMotor1 33
#define rearMotor0 4  // Used for controlling rear drive motor movement
#define rearMotor1 2

#define frontOutriggerServoPin 21
#define rearOutriggerServoPin 19
#define frontOutriggerMotor0 25  // Used for controlling front outrigger motor movement
#define frontOutriggerMotor1 26
#define rearOutriggerMotor0 12  // Used for controlling rear outrigger motor movement
#define rearOutriggerMotor1 13

#define auxAttach0 17  // Used for controlling auxillary motor or lights. In this sketch I have it settup for lights.
#define auxAttach1 16
#define auxAttach2 18  // used for controlling a second auxillary motor or some more lights. Keep in mind this will always breifly turn on when the model is powered on.
#define auxAttach3 5


#define FORWARD 1
#define BACKWARD -1
#define STOP 0

Servo rearSteeringServo;
Servo frontSteeringServo;
Servo frontOutriggerServo;
Servo rearOutriggerServo;

int lightSwitchTime = 0;
int outriggerServosTimeout = 0;
int adjustedSteeringValue = 86;
int steeringTrim = 0;
int frontOutriggerValue = 90;
int rearOutriggerValue = 90;
bool lightsOn = false;
bool lightsOn2 = false;
bool craneMode = false;
bool oRDirection = false;
bool crabWalkOn = false;

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}



void moveServo(int movement, Servo &servo, int &servoValue) {
  switch (movement) {
    case 1:
      if (servoValue >= 10 && servoValue < 170) {
        servoValue = servoValue + 5;
        servo.write(servoValue);
        delay(10);
      }
      break;
    case -1:
      if (servoValue <= 170 && servoValue > 10) {
        servoValue = servoValue - 5;
        servo.write(servoValue);
        delay(10);
      }
      break;
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr ctl) {
  //Throttle and Boom
  processThrottleAndBoom(ctl->axisY());
  //Pivot
  processPivot(ctl->axisX());
  //Boom Extension
  processExtension(ctl->axisRY());
  //Steering
  processSteering(ctl->axisRX());
  //DumpBed
  processTrimAndCables(ctl->dpad());
  //Aux
  processAux(ctl->thumbR());
  //Aux2
  processAux2(ctl->thumbL());
  //set Mode
  processMode(ctl->y());
  processORDirection(ctl->b());
  //Reverses the direction of the rear wheals so the truck can crab walk
  processCrabWalk(ctl->x());


  processRlOutrigger(ctl->l2());
  processRrOutrigger(ctl->r2());
  processFlOutrigger(ctl->l1());
  processFrOutrigger(ctl->r1());
}

void processMode(int modeValue) {
  if (modeValue == 1) {
    if (!craneMode) {
      craneMode = true;
    } else if (craneMode) {
      craneMode = false;
    }
    delay(200);
  }
}
void processORDirection(int directionValue) {
  if (directionValue == 1) {
    if (!oRDirection) {
      oRDirection = true;
    } else if (oRDirection) {
      oRDirection = false;
    }
    delay(200);
  }
}
void processCrabWalk(int crabValue) {
  if (crabValue == 1) {
    if (!crabWalkOn) {
      crabWalkOn = true;
    } else if (crabWalkOn) {
      crabWalkOn = false;
    }
    delay(200);
  }
}
void processThrottleAndBoom(int axisYValue) {
  int adjustedThrottleValue = axisYValue / 2;
  if (!craneMode) {
    moveMotor(rearMotor0, rearMotor1, adjustedThrottleValue);
    moveMotor(frontMotor0, frontMotor1, adjustedThrottleValue);
  } else {
    if (axisYValue > 60) {
      Serial.print("1,");
      Serial.println(adjustedThrottleValue);
      delay(10);
    } else if (axisYValue < -60) {
      Serial.print("2,");
      Serial.println(adjustedThrottleValue);
      delay(10);
    } else {
      Serial.println(3);
      delay(10);
    }
  }
}
void processPivot(int axisXValue) {
  int adjustedThrottleValue = axisXValue / 2;
  if (craneMode) {
    if (axisXValue > 60) {
      Serial.print("4,");
      Serial.println(adjustedThrottleValue);
      delay(10);
    } else if (axisXValue < -60) {
      Serial.print("5,");
      Serial.println(adjustedThrottleValue);
      delay(10);
    } else {
      Serial.println(6);
      delay(10);
    }
  }
}
void processExtension(int axisRYValue) {
  int adjustedThrottleValue = axisRYValue / 2;
  if (craneMode) {
    if (axisRYValue > 60) {
      Serial.print("7,");
      Serial.println(adjustedThrottleValue);
      delay(10);
    } else if (axisRYValue < -60) {
      Serial.print("8,");
      Serial.println(adjustedThrottleValue);
      delay(10);
    } else {
      Serial.println(9);
      delay(10);
    }
  }

  //Serial.println(axisRYValue);
}
void processTrimAndCables(int dpadValue) {
  if (!craneMode) {
    if (dpadValue == 1 && steeringTrim < 20) {
      steeringTrim = steeringTrim + 2;
      delay(50);
    } else if (dpadValue == 2 && steeringTrim > -20) {
      steeringTrim = steeringTrim - 2;
      delay(50);
    }
  } else {
    if (dpadValue == 1) {
      Serial.println(10);
      delay(10);
    } else if (dpadValue == 2) {
      Serial.println(11);
      delay(10);
    } else {
      Serial.println(12);
      delay(10);
    }
    if (dpadValue == 4) {
      Serial.println(13);
      delay(10);
    } else if (dpadValue == 8) {
      Serial.println(14);
      delay(10);
    } else {
      Serial.println(15);
      delay(10);
    }
  }
}

void processRlOutrigger(bool value) {
  if (value) {
    outriggerServosTimeout = millis();
    rearOutriggerServo.attach(rearOutriggerServoPin);
    rearOutriggerServo.write(80);
    if (oRDirection) {
      digitalWrite(rearOutriggerMotor0, HIGH);
      digitalWrite(rearOutriggerMotor1, LOW);
    } else {
      digitalWrite(rearOutriggerMotor0, LOW);
      digitalWrite(rearOutriggerMotor1, HIGH);
    }
  } else {
    digitalWrite(rearOutriggerMotor0, LOW);
    digitalWrite(rearOutriggerMotor1, LOW);
  }
}
void processRrOutrigger(bool value) {
  if (value) {
    outriggerServosTimeout = millis();
    rearOutriggerServo.attach(rearOutriggerServoPin);
    rearOutriggerServo.write(125);
    if (oRDirection) {
      digitalWrite(rearOutriggerMotor0, HIGH);
      digitalWrite(rearOutriggerMotor1, LOW);
    } else {
      digitalWrite(rearOutriggerMotor0, LOW);
      digitalWrite(rearOutriggerMotor1, HIGH);
    }
  }
}
void processFlOutrigger(bool value) {
  if (value) {
    outriggerServosTimeout = millis();
    frontOutriggerServo.attach(frontOutriggerServoPin);
    frontOutriggerServo.write(125);
    if (oRDirection) {
      digitalWrite(frontOutriggerMotor0, HIGH);
      digitalWrite(frontOutriggerMotor1, LOW);
    } else {
      digitalWrite(frontOutriggerMotor0, LOW);
      digitalWrite(frontOutriggerMotor1, HIGH);
    }
  } else {
    digitalWrite(frontOutriggerMotor0, LOW);
    digitalWrite(frontOutriggerMotor1, LOW);
  }
}
void processFrOutrigger(bool value) {
  if (value) {
    outriggerServosTimeout = millis();
    frontOutriggerServo.attach(frontOutriggerServoPin);
    frontOutriggerServo.write(80);
    if (oRDirection) {
      digitalWrite(frontOutriggerMotor0, HIGH);
      digitalWrite(frontOutriggerMotor1, LOW);
    } else {
      digitalWrite(frontOutriggerMotor0, LOW);
      digitalWrite(frontOutriggerMotor1, HIGH);
    }
  }
}
void processSteering(int axisRXValue) {
  // Serial.println(axisRXValue);
  adjustedSteeringValue = (90 - (axisRXValue / 17)) - steeringTrim;
  if (crabWalkOn) {
    rearSteeringServo.write(adjustedSteeringValue);
  } else {
    rearSteeringServo.write(180 - adjustedSteeringValue);
  }
  frontSteeringServo.write(adjustedSteeringValue);

  //Serial.print("Steering Value:");
  //Serial.println(adjustedSteeringValue);
}

void processAux(bool buttonValue) {
  if (buttonValue && (millis() - lightSwitchTime) > 200) {
    if (lightsOn) {
      digitalWrite(auxAttach0, LOW);
      digitalWrite(auxAttach1, LOW);
      lightsOn = false;
    } else {
      digitalWrite(auxAttach0, HIGH);
      digitalWrite(auxAttach1, LOW);
      lightsOn = true;
    }

    lightSwitchTime = millis();
  }
}
void processAux2(bool buttonValue) {
  if (buttonValue && (millis() - lightSwitchTime) > 200) {
    if (lightsOn2) {
      digitalWrite(auxAttach2, LOW);
      digitalWrite(auxAttach3, LOW);
      lightsOn2 = false;
    } else {
      digitalWrite(auxAttach2, HIGH);
      digitalWrite(auxAttach3, LOW);
      lightsOn2 = true;
    }

    lightSwitchTime = millis();
  }
}
void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (velocity > 15) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity < -15) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
  }
}
void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {


  pinMode(frontOutriggerMotor0, OUTPUT);
  pinMode(frontOutriggerMotor1, OUTPUT);
  pinMode(rearOutriggerMotor0, OUTPUT);
  pinMode(rearOutriggerMotor1, OUTPUT);
  pinMode(auxAttach0, OUTPUT);
  pinMode(auxAttach1, OUTPUT);
  pinMode(auxAttach2, OUTPUT);
  pinMode(auxAttach3, OUTPUT);
  digitalWrite(auxAttach2, LOW);
  digitalWrite(auxAttach3, LOW);


  rearSteeringServo.attach(rearSteeringServoPin);
  rearSteeringServo.write(adjustedSteeringValue);
  frontSteeringServo.attach(frontSteeringServoPin);
  frontSteeringServo.write(adjustedSteeringValue);

  frontOutriggerServo.attach(frontOutriggerServoPin);
  frontOutriggerServo.write(frontOutriggerValue);
  rearOutriggerServo.attach(rearOutriggerServoPin);
  rearOutriggerServo.write(rearOutriggerValue);
  outriggerServosTimeout = millis();

  Serial.begin(115200);
  //   put your setup code here, to run once:
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  BP32.forgetBluetoothKeys();

  BP32.enableVirtualDevice(false);
  pinMode(rearMotor0, OUTPUT);
  pinMode(rearMotor1, OUTPUT);
  pinMode(frontMotor0, OUTPUT);
  pinMode(frontMotor1, OUTPUT);
  digitalWrite(rearMotor0, LOW);
  digitalWrite(rearMotor1, LOW);
  digitalWrite(frontMotor0, LOW);
  digitalWrite(frontMotor1, LOW);
}



// Arduino loop function. Runs in CPU 1.
void loop() {
  if ((millis() - outriggerServosTimeout) > 5000){
    rearOutriggerServo.detach();
    frontOutriggerServo.detach();
  }
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  else { vTaskDelay(1); }
}
