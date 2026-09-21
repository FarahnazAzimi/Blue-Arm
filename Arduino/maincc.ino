/*
  Robotic Arm - USB Test Firmware
  --------------------------------
  Purpose: receive target angles from a PC (Python GUI) over USB Serial,
  and move 3 MG995 servos safely, one degree at a time.

  Command format sent from PC: "<id><angle 3 digits>\n"
    Example: "1090\n"  -> joint 1 (base) goes to 90 degrees
    Example: "2045\n"  -> joint 2 (shoulder) goes to 45 degrees
    Example: "9000\n"  -> HOME (safe, low-torque position - not the minimum angles)
    Example: "8000\n"  -> QUERY - Arduino replies "STATE t1,t2,t3,t4" with the
                          REAL current angles (not target), e.g. "STATE 15,90,45,0"

  Safety:
    - Every angle is clamped (constrain) to the real joint limit before use.
    - Movement is gentle: 1 degree per step, not a sudden jump.
*/

#include <Arduino.h>
#include <Servo.h>
#include <NeoSWSerial.h>

// Dedicated wireless channel to the ESP32 - separate from USB, so both
// can be used at the same time (USB stays free for the Python GUI/debugging).
// Arduino pin 4 = RX (receives from ESP32's TX2/GPIO17)
// Arduino pin 5 = TX (sends to ESP32's RX2/GPIO16)
// NOTE: using NeoSWSerial instead of plain SoftwareSerial - the standard
// SoftwareSerial library conflicts with the Servo library's timing (both
// need precise interrupts), causing lost/corrupted data with servos active.
// NeoSWSerial is built to coexist with Servo safely.
// Changed from pins 2/3 to 4/5 - pin 2 was not reliably receiving data.
NeoSWSerial espSerial(4, 5);

Servo servo1; // base
Servo servo2; // shoulder
Servo servo3; // elbow
Servo servo4; // gripper

// ---- Pin plan (confirmed with user) ----
const uint8_t PIN_SERVO1 = 9;
const uint8_t PIN_SERVO2 = 10;
const uint8_t PIN_SERVO3 = 11;
const uint8_t PIN_SERVO4 = 6; // gripper

// ---- Real safe joint limits ----
// These are the FINAL workspace ranges chosen for the pick-and-place task.
const uint8_t JOINT1_MIN = 15, JOINT1_MAX = 170; // base
const uint8_t JOINT2_MIN = 45, JOINT2_MAX = 90;  // shoulder
const uint8_t JOINT3_MIN = 25, JOINT3_MAX = 100; // elbow
const uint8_t JOINT4_MAX = 80; // gripper - user-facing max (0=closed/HOME, 80=fully open)
// IMPORTANT: physically, servo angle 0 = OPEN (confirmed by testing).
// We invert this so the USER sends 0=closed, 80=open. See serialEvent() below.

// ---- Safe power-on / HOME position ----
// theta1=15 (JOINT1_MIN), theta2=90 (JOINT2_MAX), theta3=45
const uint8_t SAFE_SHOULDER = 90;
const uint8_t SAFE_ELBOW = 45;

// current position and target position for each joint
uint8_t current1, target1;
uint8_t current2, target2;
uint8_t current3, target3;
uint8_t current4, target4; // gripper

const uint8_t STEP_DELAY_MS = 15; // pause between 1-degree steps (gentle movement)
unsigned long lastStepTime = 0;

// small buffer to build up one incoming command
char buf[6];
uint8_t bufIdx = 0;

// Applies one command (id + angle) - shared by BOTH the USB channel and
// the ESP32 channel, so they always behave identically and stay in sync.
void applyCommand(uint8_t id, int16_t angle)
{
  if (id == 1)
  {
    angle = constrain(angle, JOINT1_MIN, JOINT1_MAX);
    target1 = angle;
  }
  else if (id == 2)
  {
    angle = constrain(angle, JOINT2_MIN, JOINT2_MAX);
    target2 = angle;
  }
  else if (id == 3)
  {
    angle = constrain(angle, JOINT3_MIN, JOINT3_MAX);
    target3 = angle;
  }
  else if (id == 4) // gripper - user sends 0(closed) to 80(open), we invert to physical
  {
    angle = constrain(angle, 0, JOINT4_MAX);
    target4 = JOINT4_MAX - angle; // invert: user 0 -> physical 80 (closed), user 80 -> physical 0 (open)
  }
  else if (id == 9) // HOME command, e.g. "9000\n"
  {
    target1 = JOINT1_MIN;
    target2 = SAFE_SHOULDER;
    target3 = SAFE_ELBOW;
    target4 = JOINT4_MAX; // gripper CLOSES on home (physical 80)
  }
}

void moveTowardTarget(uint8_t &current, uint8_t target, Servo &servo)
{
  if (current == target)
    return;
  if (current < target)
    current++;
  else
    current--;
  servo.write(current);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for USB serial to be ready
  }

  espSerial.begin(9600); // ESP32 wireless bridge channel

  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  servo3.attach(PIN_SERVO3);
  servo4.attach(PIN_SERVO4);

  // start at the SAFE position (see SAFE_SHOULDER/SAFE_ELBOW above) - not
  // the minimums, which caused shoulder strain when applied suddenly
  current1 = target1 = JOINT1_MIN;
  current2 = target2 = SAFE_SHOULDER;
  current3 = target3 = SAFE_ELBOW;
  current4 = target4 = JOINT4_MAX; // gripper starts CLOSED (physical 80 = closed, since 0 = open)

  servo1.write(current1);
  servo2.write(current2);
  servo3.write(current3);
  servo4.write(current4);

  Serial.println("READY");
}

void loop()
{
  // move all joints gently, one degree per step
  if (millis() - lastStepTime >= STEP_DELAY_MS)
  {
    lastStepTime = millis();
    moveTowardTarget(current1, target1, servo1);
    moveTowardTarget(current2, target2, servo2);
    moveTowardTarget(current3, target3, servo3);
    moveTowardTarget(current4, target4, servo4);
  }

  // check for commands arriving from the ESP32 (wireless bridge)
  // SoftwareSerial has no automatic event, so we poll it here manually
  static char espBuf[6];
  static uint8_t espBufIdx = 0;

  while (espSerial.available())
  {
    char c = (char)espSerial.read();

    if (c == '\n')
    {
      if (espBufIdx == 4)
      {
        uint8_t id = espBuf[0] - '0';
        int16_t angle = (espBuf[1] - '0') * 100 + (espBuf[2] - '0') * 10 + (espBuf[3] - '0');
        Serial.print("ESP32 CMD received: id=");
        Serial.print(id);
        Serial.print(" angle=");
        Serial.println(angle);
        applyCommand(id, angle);
      }
      espBufIdx = 0;
    }
    else if (espBufIdx < 4)
    {
      espBuf[espBufIdx++] = c;
    }
  }
}

// serialEvent() runs automatically whenever USB data arrives
void serialEvent()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();

    if (c == '\n')
    {
      if (bufIdx == 4) // we expect exactly: 1 id digit + 3 angle digits
      {
        uint8_t id = buf[0] - '0';
        int16_t angle = (buf[1] - '0') * 100 + (buf[2] - '0') * 10 + (buf[3] - '0');

        if (id == 8) // QUERY command, e.g. "8000\n" - report REAL current position
        {
          Serial.print("STATE ");
          Serial.print(current1);
          Serial.print(",");
          Serial.print(current2);
          Serial.print(",");
          Serial.print(current3);
          Serial.print(",");
          Serial.println(JOINT4_MAX - current4); // convert gripper back to user-facing value
        }
        else
        {
          applyCommand(id, angle);

          Serial.print("OK ");
          Serial.print(id);
          Serial.print(" ");
          Serial.println(angle);
        }
      }
      bufIdx = 0;
    }
    else if (bufIdx < 4)
    {
      buf[bufIdx++] = c;
    }
  }
}