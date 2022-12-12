#include <AFMotor.h>
#include "InterpolationLib.h"
#define CW 2
#define CCW 1
#define DOWN 2
#define UP 1
#define FORWARD_S 2
#define BACKWARD_S 1
#define OPEN 1
#define CLOSE 2

#define SINGLE 0

// Digital Pins
#define LED_P 53
#define ESTOP_P 21
#define MSEL0_P 24
#define MSEL1_P 26
#define MSEL2_P 22
#define MSEL3_P 28
#define MANUAL_P 30
#define MANUP_P 32
#define MANDN_P 34
#define VCC 23

// Analog Pins
#define POT_BA A11
#define POT_GR A15
#define POT_SH A12
#define POT_EL A13

// Potentiometer min/maxes
#define GRIPPER_CL 360
#define GRIPPER_OP 550
#define GRIPPER_RS 10

#define BASE_DEG0 0
#define BASE_POT0 352
#define BASE_DEG1 90
#define BASE_POT1 54
#define BASE_RS 10

#define ELBOW_POT0 518
#define ELBOW_DEG0 0
#define ELBOW_POT1 161
#define ELBOW_DEG1 90
#define ELBOW_RS 5

#define SHOULDER_POT0 703
#define SHOULDER_DEG0 -45
#define SHOULDER_POT1 291
#define SHOULDER_DEG1 45
#define SHOULDER_RS 10

AF_DCMotor gripper(1, MOTOR12_1KHZ);  // The first argument stands for the number of the motors in the shield and the second
AF_DCMotor base(2, MOTOR12_1KHZ);     // one stands for the motor speed control frequency.
AF_DCMotor shoulder(3, MOTOR34_1KHZ);
AF_DCMotor elbow(4, MOTOR34_1KHZ);



const int sec = 1000;

const int constGndPin = 31;
const int constVccPin = 33;
const int contPinIn = 35;

int pot_sh = 0;
int pot_ba = 0;
int pot_el = 0;
int pot_gr = 0;

void (*softReset)(void) = 0;
const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;

int t1 = 0;
int t2 = 0;
int t3 = 0;
int pass = 0;

void setup() {
  Serial.begin(9600);
  gripper.setSpeed(255);
  base.setSpeed(255);
  shoulder.setSpeed(255);
  elbow.setSpeed(255);
  //Serial.print("Started.");

  pinMode(LED_P, OUTPUT);
  pinMode(VCC, OUTPUT);
  pinMode(ESTOP_P, INPUT);
  pinMode(MSEL0_P, INPUT);
  pinMode(MSEL1_P, INPUT);
  pinMode(MSEL2_P, INPUT);
  pinMode(MSEL3_P, INPUT);
  pinMode(MANUAL_P, INPUT);
  pinMode(MANUP_P, INPUT);
  pinMode(MANDN_P, INPUT);

  digitalWrite(VCC, HIGH);

  attachInterrupt(digitalPinToInterrupt(ESTOP_P), estop, RISING);
}

void estop() {
  while (!digitalRead(MANUP_P) || !digitalRead(MANDN_P)) {
    releaseAll();
  }
  softReset();
}
void loop() {
  if (!digitalRead(MANUAL_P)) {
    while (1) {
      // openGripper();
      // closeGripper();
      autoRun();
    }
  } else {
    while (1) {

      // Serial.print("\nInterpolate 1: " + String(interpolate(45, 0, 90, 10, 90)));
      // Serial.print("\nInterpolate 2: " + String(Interpolation::Linear(xVals, yVals, 2, 45, true)));
      // Serial.print("\nInterpolate 3: " + String(interpolate(180, 0, 90, 0, 511)));
      printPots();
      manualControl();
    }
  }
}



void printPots() {
  Serial.print("Base:");
  Serial.print(analogRead(POT_BA));
  Serial.print(",");
  Serial.print("Shoulder:");
  Serial.print(analogRead(POT_SH));
  Serial.print(",");
  Serial.print("Elbow:");
  Serial.print(analogRead(POT_EL));
  Serial.print(",");
  Serial.print("Gripper:");
  Serial.println(analogRead(POT_GR));
  delay(.1);
}
void readAnalog() {
  pot_sh = analogRead(POT_SH);
  pot_ba = analogRead(POT_BA);
  pot_el = analogRead(POT_EL);
  pot_gr = analogRead(POT_GR);
}

void manualControl() {
  if (digitalRead(MANUP_P)) {
    if (digitalRead(MSEL0_P)) {
      base.run(CW);
    }
    if (digitalRead(MSEL1_P)) {
      shoulder.run(FORWARD_S);
    }
    if (digitalRead(MSEL2_P)) {
      elbow.run(UP);
    }
    if (digitalRead(MSEL3_P)) {
      gripper.run(CLOSE);
    }
  } else if (digitalRead(MANDN_P)) {
    if (digitalRead(MSEL0_P)) {
      base.run(CCW);
    }
    if (digitalRead(MSEL1_P)) {
      shoulder.run(BACKWARD_S);
    }
    if (digitalRead(MSEL2_P)) {
      elbow.run(DOWN);
    }
    if (digitalRead(MSEL3_P)) {
      gripper.run(OPEN);
    }
  } else {
    releaseAll();
  }
}

void autoRun() {
  // Retracted position
  Serial.flush();
  moveAll(3, -50, 90);
  openGripper();

  // Wait for message from Matlab
  while (!newData){
    recvWithEndMarker();
  }
  newData = false;

  // Move to the received angles
  sscanf(receivedChars, "%d;%d;%d", &t1, &t2, &t3);

  
  Serial.println("received");
  moveAll(t1, t2, t3);
  closeGripper();

  // Move back to original position so
  // we dont hit the hoop
  // Retracted position
  moveAll(3, -50, 90);

  // Tell matlab that we think we have picked up the ball
  Serial.flush();
  Serial.println("pass");

  // Wait for ack/nack from matlab
  while (!newData){
    recvWithEndMarker();
  }
  newData = false;
  // sscanf(receivedChars, "%d", &pass);

  // Serial.flush();
  if (receivedChars[0] == '0') { return; }

  // Move to dunk position
  moveAll(3, 45, 0);
  openGripper();

}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\r';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void moveAll(int baseTarget, int shoulderTarget, int elbowTarget) {
  bool elbowReleased = true;
  bool baseReleased = true;
  bool shoulderReleased = true;
  bool ranOnce = false;

  // Determine Base Values
  double xValsBase[2] = { BASE_DEG0, BASE_DEG1 };
  double yValsBase[2] = { BASE_POT0, BASE_POT1 };
  float baseTargetPot = Interpolation::Linear(xValsBase, yValsBase, 2, baseTarget, false);

  // Determine Elbow Values
  double xValsElbow[2] = { ELBOW_DEG0, ELBOW_DEG1 };
  double yValsElbow[2] = { ELBOW_POT0, ELBOW_POT1 };
  float elbowTargetPot = Interpolation::Linear(xValsElbow, yValsElbow, 2, elbowTarget, false);
  //Serial.println("Rotating to pot " + String(targetPot));

  // Determine Shoulder Values
  double xValsShoulder[2] = { SHOULDER_DEG0, SHOULDER_DEG1 };
  double yValsShoulder[2] = { SHOULDER_POT0, SHOULDER_POT1 };
  float shoulderTargetPot = Interpolation::Linear(xValsShoulder, yValsShoulder, 2, shoulderTarget, false);

  while (!baseReleased || !shoulderReleased || !elbowReleased || !ranOnce) {
    ranOnce = true;
    // Rotate Elbow to target
    if ((analogRead(POT_EL) < elbowTargetPot - ELBOW_RS) || (analogRead(POT_EL) > elbowTargetPot + ELBOW_RS)) {
      if (elbowReleased)
      {
        elbowReleased = false;
        if (analogRead(POT_EL) > elbowTargetPot + ELBOW_RS) {
          if (ELBOW_POT0 > ELBOW_POT1) {
            elbow.run(BACKWARD);
          } else {
            elbow.run(FORWARD);
          }
        }

        if (analogRead(POT_EL) < elbowTargetPot - ELBOW_RS) {
          if (ELBOW_POT0 > ELBOW_POT1) {
            elbow.run(FORWARD);
          } else {
            elbow.run(BACKWARD);
          }
        }
      }
    } else if (!elbowReleased) {
      elbowReleased = true;
      elbow.run(RELEASE);
    }

    // Rotate base to target
    if ((analogRead(POT_BA) < baseTargetPot - BASE_RS) || (analogRead(POT_BA) > baseTargetPot + BASE_RS)) {
      if (baseReleased) {
        baseReleased = false;
        if (analogRead(POT_BA) > baseTargetPot + BASE_RS) {
          if (BASE_POT0 > BASE_POT1) {
            base.run(CCW);
          } else {
            base.run(CW);
          }
        }

        if (analogRead(POT_BA) < baseTargetPot - BASE_RS) {
          if (BASE_POT0 > BASE_POT1) {
            base.run(CW);
          } else {
            base.run(CCW);
          }
        }
        //printPots();
      }
    } else if (!baseReleased) {
      baseReleased = true;
      base.run(RELEASE);
    }

    // Rotate shoulder to target
    if ((analogRead(POT_SH) < shoulderTargetPot - SHOULDER_RS) || (analogRead(POT_SH) > shoulderTargetPot + SHOULDER_RS)) {
      if (shoulderReleased) {
        shoulderReleased = false;
        if (analogRead(POT_SH) > shoulderTargetPot + SHOULDER_RS) {
          if (BASE_POT0 > BASE_POT1) {
            shoulder.run(FORWARD_S);
          } else {
            shoulder.run(BACKWARD_S);
          }
        }

        if (analogRead(POT_SH) < shoulderTargetPot - SHOULDER_RS) {
          if (BASE_POT0 > BASE_POT1) {
            shoulder.run(BACKWARD_S);
          } else {
            shoulder.run(FORWARD_S);
          }
        }
        //printPots();
      }
    } else if (!shoulderReleased) {
      shoulderReleased = true;
      shoulder.run(RELEASE);
    }
  }
}


void rotateElbow(int target_degree) {
  double xVals[2] = { ELBOW_DEG0, ELBOW_DEG1 };
  double yVals[2] = { ELBOW_POT0, ELBOW_POT1 };
  float targetPot = Interpolation::Linear(xVals, yVals, 2, target_degree, false);
  //Serial.println("Rotating to pot " + String(targetPot));
  while ((analogRead(POT_EL) < targetPot - ELBOW_RS) || (analogRead(POT_EL) > targetPot + ELBOW_RS)) {

    if (analogRead(POT_EL) > targetPot + ELBOW_RS) {
      if (ELBOW_POT0 > ELBOW_POT1) {
        elbow.run(BACKWARD);
      } else {
        elbow.run(FORWARD);
      }
    }

    if (analogRead(POT_EL) < targetPot - ELBOW_RS) {
      if (ELBOW_POT0 > ELBOW_POT1) {
        elbow.run(FORWARD);
      } else {
        elbow.run(BACKWARD);
      }
    }
    //printPots();
  }
  elbow.run(RELEASE);
  //Serial.println("Reached target.");
  delay(250);
}

void rotateBase(int target_degree) {
  double xVals[2] = { BASE_DEG0, BASE_DEG1 };
  double yVals[2] = { BASE_POT0, BASE_POT1 };
  float targetPot = Interpolation::Linear(xVals, yVals, 2, target_degree, false);
  //Serial.println("Rotating to pot " + String(targetPot));
  while ((analogRead(POT_BA) < targetPot - BASE_RS) || (analogRead(POT_BA) > targetPot + BASE_RS)) {

    if (analogRead(POT_BA) > targetPot + BASE_RS) {
      if (BASE_POT0 > BASE_POT1) {
        base.run(CCW);
      } else {
        base.run(CW);
      }
    }

    if (analogRead(POT_BA) < targetPot - BASE_RS) {
      if (BASE_POT0 > BASE_POT1) {
        base.run(CW);
      } else {
        base.run(CCW);
      }
    }
    //printPots();
  }
  base.run(RELEASE);
  //Serial.println("Reached target.");
  delay(250);
}

void rotateShoulder(int target_degree) {
  double xVals[2] = { SHOULDER_DEG0, SHOULDER_DEG1 };
  double yVals[2] = { SHOULDER_POT0, SHOULDER_POT1 };
  float targetPot = Interpolation::Linear(xVals, yVals, 2, target_degree, false);
  //Serial.println("Rotating to pot " + String(targetPot));
  while ((analogRead(POT_SH) < targetPot - SHOULDER_RS) || (analogRead(POT_SH) > targetPot + SHOULDER_RS)) {

    if (analogRead(POT_SH) > targetPot + SHOULDER_RS) {
      if (BASE_POT0 > BASE_POT1) {
        shoulder.run(FORWARD_S);
      } else {
        shoulder.run(BACKWARD_S);
      }
    }

    if (analogRead(POT_SH) < targetPot - SHOULDER_RS) {
      if (BASE_POT0 > BASE_POT1) {
        shoulder.run(BACKWARD_S);
      } else {
        shoulder.run(FORWARD_S);
      }
    }
    //printPots();
  }
  shoulder.run(RELEASE);
  //Serial.println("Reached target.");
  delay(250);
}

void closeGripper() {
  int curGripper = analogRead(POT_GR);
  int newGripper = 0;
  int counter = 0;
  float err = 0;

  while ((analogRead(POT_GR) < GRIPPER_CL - GRIPPER_RS) || (analogRead(POT_GR) > GRIPPER_CL + GRIPPER_RS)) {
    if (analogRead(POT_GR) > GRIPPER_CL - GRIPPER_RS) {
      if (GRIPPER_CL < GRIPPER_OP) {
        gripper.run(CLOSE);
      } else {
        gripper.run(OPEN);
      }
    }
    if (analogRead(POT_GR) > GRIPPER_CL + GRIPPER_RS) {
      if (GRIPPER_CL > GRIPPER_CL) {
        gripper.run(OPEN);
      } else {
        gripper.run(CLOSE);
      }
    }
    newGripper = analogRead(POT_GR);
    err = (curGripper - newGripper) / curGripper;
    if (err < .05) {
      counter ++;
    }
    if (counter > 800) {
      break;
    }
    curGripper = newGripper;
    //printPots();
  }
  gripper.run(RELEASE);
  delay(250);
}

void openGripper() {
  while ((analogRead(POT_GR) < GRIPPER_OP - GRIPPER_RS) || (analogRead(POT_GR) > GRIPPER_OP + GRIPPER_RS)) {
    if (analogRead(POT_GR) < GRIPPER_OP - GRIPPER_RS) {
      if (GRIPPER_OP > GRIPPER_CL) {
        gripper.run(OPEN);
      } else {
        gripper.run(CLOSE);
      }
    }
    if (analogRead(POT_GR) > GRIPPER_OP + GRIPPER_RS) {
      if (GRIPPER_OP > GRIPPER_CL) {
        gripper.run(CLOSE);
      } else {
        gripper.run(OPEN);
      }
    }
    //printPots();
  }
  gripper.run(RELEASE);
  delay(250);
}

void releaseAll() {
  gripper.run(RELEASE);
  base.run(RELEASE);
  shoulder.run(RELEASE);
  elbow.run(RELEASE);
}