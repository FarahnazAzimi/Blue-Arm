/*
  Robotic Arm - ESP32-ONLY Firmware
  ------------------------------------
  This replaces BOTH main.cpp (Arduino) AND the old esp32_wifi_bridge.ino.
  The Arduino Uno is no longer used - the ESP32 drives all 4 servos
  directly (confirmed reliable at 3.3V logic, no driver module needed).

  CONTROL METHODS (all work at the same time):
    1. USB Serial (same protocol as before) - for the Python GUI /
       pick_and_place.py, works exactly like the old Arduino did.
    2. WiFi webpage - sliders, HOME, and Pick & Place, same as before.

  WIRING (signal wires - power/GND same PSU rail as always)
    Servo 1 (base)     -> GPIO 32
    Servo 2 (shoulder) -> GPIO 16
    Servo 3 (elbow)    -> GPIO 27
    Servo 4 (gripper)  -> GPIO 19
  These are spread across different areas of the board on purpose, to
  reduce solder-bridge risk between adjacent signal pins. Double check
  the physical spacing on YOUR specific board before soldering - if any
  two look too close together, just change the numbers below (PIN1-4)
  to any other free GPIO; nothing else in the code depends on which
  exact pins these are.

  Requires: "ESP32Servo" library (Library Manager)
*/

#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ================= PINS =================
// Spread across both header rows for physical separation during
// soldering - reduces risk of an accidental solder bridge/short
// between adjacent signal pins. See wiring note above.
const int PIN1 = 3, PIN2 = 4, PIN3 = 5, PIN4 = 7;

// ================= JOINT LIMITS (must match kinematics.py) =================
const uint8_t J1_MIN = 15, J1_MAX = 170; // base
const uint8_t J2_MIN = 45, J2_MAX = 90;  // shoulder
const uint8_t J3_MIN = 25, J3_MAX = 100; // elbow
const uint8_t J4_MAX = 80;               // gripper (0=closed/HOME, 80=open)

// Safe power-on / HOME position (low-torque, confirmed by testing)
const uint8_t SAFE_SHOULDER = 90;
const uint8_t SAFE_ELBOW = 45;

// ================= SERVOS =================
Servo servo1, servo2, servo3, servo4;
uint8_t current1, target1;
uint8_t current2, target2;
uint8_t current3, target3;
uint8_t current4, target4;

const uint8_t STEP_DELAY_MS = 1; // gentle movement for the 3 heavier arm joints (near-instant, still gradual for safety)
const uint8_t GRIPPER_STEP_DELAY_MS = 3; // gripper is light (SG90) - already fast, unchanged
unsigned long lastStepTime = 0;
unsigned long lastGripperStepTime = 0;

// ================= WIFI =================
const char *AP_SSID = "RoboticArm";
const char *AP_PASSWORD = "arm12345";
WebServer server(80);

// ================= KINEMATICS (must match kinematics.py) =================
const float GROUND_OFFSET = 5.0;
const float L1 = 8.0;
const float L2 = 21.0;
const float L3 = 28.0;
const float THETA1_OFFSET = 9.0; // NEW - degrees: real base angle = commanded - this
const float THETA2_OFFSET = 15.5;
const float THETA3_OFFSET = 49.7;

// Empirical position correction, found by testing a real target and
// measuring where the gripper actually closed (see kinematics.py notes).
// Real result was: X +5cm too far, Y -4cm too short, Z +7cm too high.
const float POS_CORRECTION_X = -5.0;
const float POS_CORRECTION_Y = 4.0;
const float POS_CORRECTION_Z = -11.0; // updated from -7.0 using new wide-angle test data

struct IKResult
{
  bool valid;
  float theta1, theta2, theta3;
};

IKResult inverseKinematics(float x, float y, float z)
{
  // Apply the empirical position correction FIRST, same as kinematics.py
  x = x + POS_CORRECTION_X;
  y = y + POS_CORRECTION_Y;
  z = z + POS_CORRECTION_Z;

  IKResult res;
  res.theta1 = degrees(atan2(y, x)) + THETA1_OFFSET;

  float r = sqrt(x * x + y * y);
  float z_rel = z - GROUND_OFFSET - L1;
  float D = sqrt(r * r + z_rel * z_rel);

  if (D > (L2 + L3) || D < fabs(L2 - L3))
  {
    res.valid = false;
    return res;
  }

  float cos_gamma = (L2 * L2 + L3 * L3 - D * D) / (2 * L2 * L3);
  cos_gamma = constrain(cos_gamma, -1.0f, 1.0f);
  float gamma = acos(cos_gamma);
  float theta3_real_bend = 180 - degrees(gamma);

  float cos_beta = (L2 * L2 + D * D - L3 * L3) / (2 * L2 * D);
  cos_beta = constrain(cos_beta, -1.0f, 1.0f);
  float beta = degrees(acos(cos_beta));
  float theta2_real = degrees(atan2(z_rel, r)) + beta;

  res.theta2 = theta2_real + THETA2_OFFSET;
  res.theta3 = theta3_real_bend - THETA3_OFFSET;
  res.valid = true;
  return res;
}

bool withinLimits(float t1, float t2, float t3)
{
  return (t1 >= J1_MIN && t1 <= J1_MAX) &&
         (t2 >= J2_MIN && t2 <= J2_MAX) &&
         (t3 >= J3_MIN && t3 <= J3_MAX);
}

// ================= SHARED COMMAND LOGIC =================
// Applies one command (id + angle). Used by BOTH USB serial and the
// webpage, so behavior always stays identical between control methods.
void applyCommand(uint8_t id, int16_t angle)
{
  if (id == 1)
  {
    angle = constrain(angle, (int16_t)J1_MIN, (int16_t)J1_MAX);
    target1 = angle;
  }
  else if (id == 2)
  {
    angle = constrain(angle, (int16_t)J2_MIN, (int16_t)J2_MAX);
    target2 = angle;
  }
  else if (id == 3)
  {
    angle = constrain(angle, (int16_t)J3_MIN, (int16_t)J3_MAX);
    target3 = angle;
  }
  else if (id == 4) // gripper - user sends 0(closed) to 80(open), inverted to physical
  {
    angle = constrain(angle, (int16_t)0, (int16_t)J4_MAX);
    target4 = J4_MAX - angle;
    current4 = target4;      // gripper has low torque needs - move it INSTANTLY
    servo4.write(current4);  // (arm joints still move gradually for safety, gripper does not)
  }
  else if (id == 9) // HOME
  {
    target1 = J1_MIN;
    target2 = SAFE_SHOULDER;
    target3 = SAFE_ELBOW;
    target4 = J4_MAX;
  }
}

void moveTowardTarget(uint8_t &cur, uint8_t tgt, Servo &s)
{
  if (cur == tgt)
    return;
  if (cur < tgt)
    cur++;
  else
    cur--;
  s.write(cur);
}

// Blocking move used by the pick-and-place sequence - steps smoothly
// and only returns once ALL 4 joints (including the gripper) have
// arrived at their targets.
void moveToAnglesBlocking(float t1, float t2, float t3)
{
  applyCommand(1, round(t1));
  applyCommand(2, round(t2));
  applyCommand(3, round(t3));

  unsigned long lastArmStep = 0;
  unsigned long lastGripStep = 0;

  while (current1 != target1 || current2 != target2 || current3 != target3 || current4 != target4)
  {
    unsigned long now = millis();
    if (now - lastArmStep >= STEP_DELAY_MS)
    {
      lastArmStep = now;
      moveTowardTarget(current1, target1, servo1);
      moveTowardTarget(current2, target2, servo2);
      moveTowardTarget(current3, target3, servo3);
    }
    if (now - lastGripStep >= GRIPPER_STEP_DELAY_MS)
    {
      lastGripStep = now;
      moveTowardTarget(current4, target4, servo4);
    }
  }
}

// Blocking wait for the gripper alone - used right after opening/closing
// it, so we don't move on until it has actually finished (instead of a
// fixed delay() that doesn't guarantee the gripper reached its target).
void waitForGripper()
{
  while (current4 != target4)
  {
    moveTowardTarget(current4, target4, servo4);
    delay(GRIPPER_STEP_DELAY_MS);
  }
}

// ================= USB SERIAL (same protocol as the old Arduino) =================
char usbBuf[6];
uint8_t usbIdx = 0;

void pollUSBSerial()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n')
    {
      if (usbIdx == 4)
      {
        uint8_t id = usbBuf[0] - '0';
        int16_t angle = (usbBuf[1] - '0') * 100 + (usbBuf[2] - '0') * 10 + (usbBuf[3] - '0');

        if (id == 8) // QUERY - report real current position
        {
          Serial.print("STATE ");
          Serial.print(current1);
          Serial.print(",");
          Serial.print(current2);
          Serial.print(",");
          Serial.print(current3);
          Serial.print(",");
          Serial.println(J4_MAX - current4);
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
      usbIdx = 0;
    }
    else if (usbIdx < 4)
    {
      usbBuf[usbIdx++] = c;
    }
  }
}

// ================= BLUETOOTH (Classic - works natively with Android) =================
// Two message formats, both ending in '\n':
//   "<id><angle 3 digits>"      example: "1090"   - normal joint command
//   "P<px>,<py>,<pz>,<lx>,<ly>,<lz>"  example: "P10,20,5,10,30,5" - pick-and-place
// Buffer is bigger now (64 chars) to fit the longer pick-and-place message.
char btBuf[64];
uint8_t btIdx = 0;

// Splits "10,20,5,10,30,5" into 6 float values. Returns true if exactly
// 6 numbers were found.
bool parseSixFloats(const String &payload, float out[6])
{
  int found = 0;
  int start = 0;
  for (int i = 0; i <= (int)payload.length() && found < 6; i++)
  {
    if (i == (int)payload.length() || payload[i] == ',')
    {
      out[found++] = payload.substring(start, i).toFloat();
      start = i + 1;
    }
  }
  return found == 6;
}

void pollBluetooth()
{
  while (SerialBT.available())
  {
    char c = SerialBT.read();
    if (c == '\n')
    {
      btBuf[btIdx] = '\0'; // null-terminate so we can treat it as a String safely

      if (btIdx == 4 && isDigit(btBuf[0]))
      {
        // ---- Normal joint command: "<id><angle 3 digits>" ----
        uint8_t id = btBuf[0] - '0';
        int16_t angle = (btBuf[1] - '0') * 100 + (btBuf[2] - '0') * 10 + (btBuf[3] - '0');

        if (id == 8) // QUERY - report real current position
        {
          SerialBT.print("STATE ");
          SerialBT.print(current1);
          SerialBT.print(",");
          SerialBT.print(current2);
          SerialBT.print(",");
          SerialBT.print(current3);
          SerialBT.print(",");
          SerialBT.println(J4_MAX - current4);
        }
        else
        {
          applyCommand(id, angle);
          //SerialBT.print("OK ");
          //SerialBT.print(id);
          //SerialBT.print(" ");
          //SerialBT.println(angle);
        }
      }
      else if (btIdx > 0 && btBuf[0] == 'P')
      {
        // ---- Pick-and-place command: "P px,py,pz,lx,ly,lz" ----
        float vals[6];
        if (parseSixFloats(String(btBuf + 1), vals))
        {
          String result = runPickPlaceSequence(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);
          SerialBT.println(result);
        }
        else
        {
          SerialBT.println("Bad pick-and-place format - need 6 numbers");
        }
      }

      btIdx = 0;
    }
    else if (btIdx < sizeof(btBuf) - 1)
    {
      btBuf[btIdx++] = c;
    }
  }
}

// ================= WEBPAGE =================
struct JointInfo
{
  int id;
  int minAngle;
  int maxAngle;
  const char *name;
};

JointInfo joints[] = {
    {1, J1_MIN, J1_MAX, "Base"},
    {2, J2_MIN, J2_MAX, "Shoulder"},
    {3, J3_MIN, J3_MAX, "Elbow"},
    {4, 0, J4_MAX, "Gripper"},
};

struct HomeValue
{
  int id;
  int angle;
};
HomeValue homeValues[] = {
    {1, (int)J1_MIN},
    {2, (int)SAFE_SHOULDER},
    {3, (int)SAFE_ELBOW},
    {4, 0},
};

String buildPage()
{
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Robotic Arm Control</title>";
  html += "<style>";
  html += "* { box-sizing: border-box; }";
  html += "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;";
  html += "  background:#0b1220;color:#e2e8f0;padding:20px;margin:0;}";
  html += "h2{font-weight:600;letter-spacing:0.3px;margin:0 0 20px 0;color:#f1f5f9;}";
  html += ".card{background:#141d33;border:1px solid #24304d;border-radius:14px;";
  html += "  padding:18px;margin-bottom:14px;}";
  html += ".card h3{margin:0 0 12px 0;font-size:15px;font-weight:500;color:#93a5c9;";
  html += "  display:flex;justify-content:space-between;}";
  html += ".card h3 span{color:#5ec8f8;font-weight:700;font-variant-numeric:tabular-nums;}";
  html += "input[type=range]{-webkit-appearance:none;width:100%;height:8px;border-radius:5px;";
  html += "  background:#24304d;outline:none;}";
  html += "input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:26px;height:26px;";
  html += "  border-radius:50%;background:#5ec8f8;cursor:pointer;box-shadow:0 2px 6px rgba(94,200,248,0.5);}";
  html += "input[type=range]::-moz-range-thumb{width:26px;height:26px;border:none;border-radius:50%;";
  html += "  background:#5ec8f8;cursor:pointer;box-shadow:0 2px 6px rgba(94,200,248,0.5);}";
  html += "#home{width:100%;padding:16px;background:#e0442e;color:white;border:none;";
  html += "  border-radius:12px;font-size:16px;font-weight:600;letter-spacing:0.3px;margin-top:6px;}";
  html += "#home:active{background:#c23a26;}";
  html += ".pp-row{display:flex;gap:8px;margin-bottom:8px;}";
  html += ".pp-row input{flex:1;min-width:0;background:#0b1220;border:1px solid #24304d;";
  html += "  color:#e2e8f0;border-radius:8px;padding:10px;font-size:14px;}";
  html += ".pp-label{font-size:12px;color:#93a5c9;margin:10px 0 4px 0;}";
  html += "#runpp{width:100%;padding:16px;background:#5ec8f8;color:#0b1220;border:none;";
  html += "  border-radius:12px;font-size:16px;font-weight:700;letter-spacing:0.3px;margin-top:10px;}";
  html += "#runpp:disabled{background:#3a4a6b;color:#93a5c9;}";
  html += "#ppstatus{margin-top:10px;font-size:14px;color:#93a5c9;min-height:20px;}";
  html += ".pp-status{font-size:13px;margin-bottom:10px;padding:8px 10px;border-radius:8px;min-height:16px;}";
  html += ".pp-ok{background:rgba(52,199,89,0.15);color:#34d399;}";
  html += ".pp-bad{background:rgba(224,68,46,0.15);color:#f87171;}";
  html += "</style></head><body>";
  html += "<h2>&#129302; Robotic Arm Control</h2>";

  for (JointInfo &j : joints)
  {
    html += "<div class='card'>";
    html += "<h3>" + String(j.name) + " <span id='val" + String(j.id) + "'></span></h3>";
    html += "<input type='range' min='" + String(j.minAngle) + "' max='" + String(j.maxAngle) +
            "' id='slider" + String(j.id) + "' oninput='moveJoint(" + String(j.id) + ",this.value)'>";
    html += "</div>";
  }

  html += "<button id='home' onclick='goHome()'>HOME</button>";

  html += "<div class='card'>";
  html += "<h3>Pick &amp; Place</h3>";
  html += "<div class='pp-label'>Pick position (x, y, z cm)</div>";
  html += "<div class='pp-row'>";
  html += "<input type='text' id='px' placeholder='x' value='10' oninput='validatePoint(\"p\")'>";
  html += "<input type='text' id='py' placeholder='y' value='20' oninput='validatePoint(\"p\")'>";
  html += "<input type='text' id='pz' placeholder='z' value='5' oninput='validatePoint(\"p\")'>";
  html += "</div>";
  html += "<div id='pstatus' class='pp-status'></div>";
  html += "<div class='pp-label'>Place position (x, y, z cm)</div>";
  html += "<div class='pp-row'>";
  html += "<input type='text' id='lx' placeholder='x' value='10' oninput='validatePoint(\"l\")'>";
  html += "<input type='text' id='ly' placeholder='y' value='30' oninput='validatePoint(\"l\")'>";
  html += "<input type='text' id='lz' placeholder='z' value='5' oninput='validatePoint(\"l\")'>";
  html += "</div>";
  html += "<div id='lstatus' class='pp-status'></div>";
  html += "<button id='runpp' onclick='runPickPlace()'>Run Pick &amp; Place</button>";
  html += "<div id='ppstatus'></div>";
  html += "</div>";

  html += "<script>";
  html += "function moveJoint(id, angle){";
  html += "  document.getElementById('val'+id).innerText = angle + '\u00b0';";
  html += "  fetch('/set?id=' + id + '&angle=' + angle);";
  html += "}";
  html += "const homeValues = {";
  for (HomeValue &h : homeValues)
  {
    html += String(h.id) + ":" + String(h.angle) + ",";
  }
  html += "};";
  html += "function goHome(){";
  html += "  fetch('/home');";
  html += "  for (const id in homeValues){";
  html += "    document.getElementById('slider'+id).value = homeValues[id];";
  html += "    document.getElementById('val'+id).innerText = homeValues[id] + '\u00b0';";
  html += "  }";
  html += "}";
  html += "window.onload = goHome;";

  html += "function runPickPlace(){";
  html += "  validatePoint('p'); validatePoint('l');";
  html += "  const pBad = document.getElementById('pstatus').className.includes('pp-bad');";
  html += "  const lBad = document.getElementById('lstatus').className.includes('pp-bad');";
  html += "  const status = document.getElementById('ppstatus');";
  html += "  if (pBad || lBad){";
  html += "    status.innerText = 'Fix the red warning(s) above before running.';";
  html += "    return;";
  html += "  }";
  html += "  const btn = document.getElementById('runpp');";
  html += "  const params = ['px','py','pz','lx','ly','lz'];";
  html += "  const q = params.map(id => id + '=' + document.getElementById(id).value).join('&');";
  html += "  btn.disabled = true;";
  html += "  status.innerText = 'Running... (takes about 20-30 seconds)';";
  html += "  fetch('/pickplace?' + q).then(r => r.text()).then(msg => {";
  html += "    status.innerText = msg;";
  html += "    btn.disabled = false;";
  html += "    goHome();";
  html += "  }).catch(() => {";
  html += "    status.innerText = 'Request failed - check connection.';";
  html += "    btn.disabled = false;";
  html += "  });";
  html += "}";

  html += "const K = {GROUND:5, L1:8, L2:21, L3:28, T1OFF:9, T2OFF:15.5, T3OFF:49.7,";
  html += "  CX:-5, CY:4, CZ:-11,";
  html += "  J1MIN:15, J1MAX:170, J2MIN:45, J2MAX:90, J3MIN:25, J3MAX:100};";
  html += "function ik(x, y, z){";
  html += "  x = x + K.CX; y = y + K.CY; z = z + K.CZ;";
  html += "  const theta1 = Math.atan2(y, x) * 180 / Math.PI + K.T1OFF;";
  html += "  const r = Math.hypot(x, y);";
  html += "  const zRel = z - K.GROUND - K.L1;";
  html += "  const D = Math.hypot(r, zRel);";
  html += "  if (D > (K.L2 + K.L3) || D < Math.abs(K.L2 - K.L3)) return null;";
  html += "  let cosGamma = (K.L2*K.L2 + K.L3*K.L3 - D*D) / (2*K.L2*K.L3);";
  html += "  cosGamma = Math.max(-1, Math.min(1, cosGamma));";
  html += "  const gamma = Math.acos(cosGamma);";
  html += "  const theta3Bend = 180 - gamma * 180 / Math.PI;";
  html += "  let cosBeta = (K.L2*K.L2 + D*D - K.L3*K.L3) / (2*K.L2*D);";
  html += "  cosBeta = Math.max(-1, Math.min(1, cosBeta));";
  html += "  const beta = Math.acos(cosBeta) * 180 / Math.PI;";
  html += "  const theta2Real = Math.atan2(zRel, r) * 180 / Math.PI + beta;";
  html += "  const theta2 = theta2Real + K.T2OFF;";
  html += "  const theta3 = theta3Bend - K.T3OFF;";
  html += "  return {theta1, theta2, theta3};";
  html += "}";
  html += "function withinLimits(a){";
  html += "  return a.theta1>=K.J1MIN && a.theta1<=K.J1MAX &&";
  html += "         a.theta2>=K.J2MIN && a.theta2<=K.J2MAX &&";
  html += "         a.theta3>=K.J3MIN && a.theta3<=K.J3MAX;";
  html += "}";
  html += "function validatePoint(prefix){";
  html += "  const x = parseFloat(document.getElementById(prefix+'x').value) || 0;";
  html += "  const y = parseFloat(document.getElementById(prefix+'y').value) || 0;";
  html += "  const z = parseFloat(document.getElementById(prefix+'z').value) || 0;";
  html += "  const el = document.getElementById(prefix + 'status');";
  html += "  const base = (y > 0) ? ik(x, y, z) : null;";
  html += "  if (base === null || !withinLimits(base)){";
  html += "    el.className = 'pp-status pp-bad';";
  html += "    el.innerText = '\u2717 Unreachable';";
  html += "    return;";
  html += "  }";
  html += "  el.className = 'pp-status pp-ok';";
  html += "  el.innerText = '\u2713 Reachable';";
  html += "}";
  html += "window.addEventListener('load', () => { validatePoint('p'); validatePoint('l'); });";
  html += "</script></body></html>";

  return html;
}

void handleRoot()
{
  server.send(200, "text/html", buildPage());
}

void handleSet()
{
  if (server.hasArg("id") && server.hasArg("angle"))
  {
    applyCommand(server.arg("id").toInt(), server.arg("angle").toInt());
    server.send(200, "text/plain", "OK");
  }
  else
  {
    server.send(400, "text/plain", "Missing id or angle");
  }
}

void handleHome()
{
  applyCommand(9, 0);
  server.send(200, "text/plain", "OK");
}

void handleState()
{
  String json = "{\"1\":" + String(current1) + ",\"2\":" + String(current2) +
                ",\"3\":" + String(current3) + ",\"4\":" + String(J4_MAX - current4) + "}";
  server.send(200, "application/json", json);
}

// Runs the full pick-and-place sequence and returns a result message.
// Shared by BOTH the HTTP handler and the Bluetooth command handler,
// so there is only ONE place this logic lives - keeps them always
// in sync and avoids duplicating (and possibly mismatching) the logic.
String runPickPlaceSequence(float px, float py, float pz, float lx, float ly, float lz)

{

IKResult atPick = inverseKinematics(px, py, pz);

IKResult atPlace = inverseKinematics(lx, ly, lz);

IKResult *steps[2] = {&atPick, &atPlace};

const char *names[2] = {"pick", "place"};

for (int i = 0; i < 2; i++)

{

if (!steps[i]->valid || !withinLimits(steps[i]->theta1, steps[i]->theta2, steps[i]->theta3))

{

  return String("Target '") + names[i] + "' is unreachable.";

}

}

applyCommand(4, 80); // open gripper - starts NOW, in parallel with the move below

moveToAnglesBlocking(atPick.theta1, atPick.theta2, atPick.theta3);

// ================= WAIT 2 SECONDS AT PICK =================

delay(2000);

// =========================================================

applyCommand(4, 0); // close gripper (grab, staying at pick position)

waitForGripper();

moveToAnglesBlocking(atPlace.theta1, atPlace.theta2, atPlace.theta3);

// ================= WAIT 2 SECONDS AT PLACE =================

delay(2000);

// ==========================================================

applyCommand(4, 80); // release

waitForGripper();

applyCommand(9, 0); // HOME (sets arm targets AND gripper target=closed, together)

moveToAnglesBlocking(target1, target2, target3); // arm returns home WHILE gripper closes, in parallel

return "Pick and place complete";

}

void handlePickPlace()
{
  if (!(server.hasArg("px") && server.hasArg("py") && server.hasArg("pz") &&
        server.hasArg("lx") && server.hasArg("ly") && server.hasArg("lz")))
  {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }

  float px = server.arg("px").toFloat();
  float py = server.arg("py").toFloat();
  float pz = server.arg("pz").toFloat();
  float lx = server.arg("lx").toFloat();
  float ly = server.arg("ly").toFloat();
  float lz = server.arg("lz").toFloat();

  String result = runPickPlaceSequence(px, py, pz, lx, ly, lz);
  server.send(200, "text/plain", result);
}

// ================= SETUP / LOOP =================
void setup()
{
  Serial.begin(115200);

  servo1.attach(PIN1);
  servo2.attach(PIN2);
  servo3.attach(PIN3);
  servo4.attach(PIN4);

  // Boot into the safe HOME position
  current1 = target1 = J1_MIN;
  current2 = target2 = SAFE_SHOULDER;
  current3 = target3 = SAFE_ELBOW;
  current4 = target4 = J4_MAX; // closed

  servo1.write(current1);
  servo2.write(current2);
  servo3.write(current3);
  servo4.write(current4);

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("WiFi hotspot started: ");
  Serial.println(AP_SSID);
  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());

  SerialBT.begin("RoboticArm"); // Bluetooth device name - shows up when pairing
  Serial.println("Bluetooth started - pair with 'RoboticArm' from your phone.");

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/home", handleHome);
  server.on("/state", handleState);
  server.on("/pickplace", handlePickPlace);
  server.begin();

  Serial.println("READY");
}

void loop()
{
  server.handleClient();
  pollUSBSerial();
  pollBluetooth();

  if (millis() - lastStepTime >= STEP_DELAY_MS)
  {
    lastStepTime = millis();
    moveTowardTarget(current1, target1, servo1);
    moveTowardTarget(current2, target2, servo2);
    moveTowardTarget(current3, target3, servo3);
  }

  if (millis() - lastGripperStepTime >= GRIPPER_STEP_DELAY_MS)
  {
    lastGripperStepTime = millis();
    moveTowardTarget(current4, target4, servo4);
  }
}
