/*
Rebelia-Robotic-Firmware is the control software for the Rebelia Robotic Hand, a robotic end-effector device (see https://www.robotgarage.org).

The Copyright Notice
Copyright (C)  2023 Vittorio Lumare

The License Notices
    This file is part of Rebelia-Robotic-Firmware.

    Rebelia-Robotic-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

    Rebelia-Robotic-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with Rebelia-Hand-Firmware. If not, see <https://www.gnu.org/licenses/>. 
*/

#include <SCServo.h>
#include <BTAddress.h>
#include <BTAdvertisedDevice.h>
#include <BTScan.h>
#include <BluetoothSerial.h>
#include "fingers_controller.h"

#define S_RXD 18
#define S_TXD 19
#define BLUETOOTH true

static const bool SHOW_FEEDBACK = false;
static const bool CALIBRATE_CENTER = false;

static const int INDEX_FLEX_MOTOR = 1;
static const int MIDDLE_FLEX_MOTOR = 2;
static const int RING_LITTLE_FLEX_MOTOR = 3;
static const int THUMB_FLEX_MOTOR = 4;
static const int THUMB_ROT_MOTOR = 5;

BluetoothSerial SerialBT;
SMS_STS st;
FingersController fc;

int multiarray1[2][2] = { { 0, 1 }, { 2, 3 } };
int multiarray2[2][3] = { { 4, 5, 6 }, { 7, 8, 9 } };
void testMultiDimArray(int** multiarray, int arrayrows, int arraycols) {

  for (int i = 0; i < arrayrows; i++) {
    for (int j = 0; j < arraycols; j++) {
      SerialBT.println(multiarray[arrayrows][arraycols]);
    }
  }
};

void servoIdDiscovery() {
  SerialBT.println("Servo ID Discovery");
  for (int i = 0; i <= 5; i++) {
    int ID = st.Ping(i);
    if (ID != -1) {
      SerialBT.print("Servo ID:");
      SerialBT.println(ID, DEC);
      delay(100);
    } else {
      SerialBT.print("Servo ID:");
      SerialBT.print(ID, DEC);
      SerialBT.println(" ERROR!");
      delay(2000);
    }
  }
}

void setup() {

  Serial.begin(115200);
  SerialBT.begin();
  SerialBT.setTimeout(1);

  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);  // custom serial port
  st.pSerial = &Serial1;
  while (!Serial1) {
    SerialBT.println("ST3215 serial Not ready");
    delay(100);
  }

  //delay(10000);  // Initial pause
  SerialBT.println("ST3215 serial Ready!");
  // servoIdDiscovery();

  //fc.moveAllFingersToMiddlePosition()
  if (CALIBRATE_CENTER) {
    fc.calibrateCenterOfRange(FingersController::INDEX_FLEX_MOTOR);
    fc.calibrateCenterOfRange(FingersController::MIDDLE_FLEX_MOTOR);
    fc.calibrateCenterOfRange(FingersController::RING_LITTLE_FLEX_MOTOR);
    fc.calibrateCenterOfRange(FingersController::THUMB_FLEX_MOTOR);
    SerialBT.println("All servos center position calibrated!");
    while (1)
      ;
  }

  //Changing Motor ID
  //fc.changeID(5, RING_LITTLE_FLEX_MOTOR);

  // Hand Calibration
  fc.performTendonCalibration();
}

FingersController::GraspType g_grasp_type = FingersController::GraspType::MONKEY;
FingersController::GraspType g_new_grasp_type = g_grasp_type;
int g_closurePercent = 0;
int g_newClosurePercent = 0;
int g_load_limit[4] = { 308, 308, 308, 264 };
bool g_preparation = true;
bool g_closure = false;
bool g_motor_enabled[5] = { true, true, true, true, true };
enum class FSM {
  Control,
  TendonInstall,
  Testing,
  DoNothing
};
FSM gFsmState = FSM::Control;

void processStringCmd(const String& cmd);
void prepareGrasp();
void limitLoad();
int sign(int val);

int sign(int val) {
  return val / abs(val);
}

int gMaxOverLoadCount = 40;
int gOverLoadCounters[5];  // 0: index, ..

void limitLoad() {

  if (DEBUG) { SerialBT.println("LIMITING LOAD.."); }

  for (int idx = FingersController::VectorIdx::Index; idx <= FingersController::VectorIdx::ThumbRot; idx++) {
    auto id = fc.getMotorIdByVectorIndex((FingersController::VectorIdx)idx);
    if (fc.isMoving(id)) {
      gOverLoadCounters[idx] = 0;  //reset
      if (DEBUG) { SerialBT.printf("Id %d reset count\n", id); }
      continue;
    }
    delay(200);
    auto load = fc.getLoad(id);
    if (abs(load) > 220) {
      gOverLoadCounters[idx]++;
      if (DEBUG) { SerialBT.printf("Id %d overLoadCount: %d\n", id, gOverLoadCounters[idx]); }
    }
    while (gOverLoadCounters[idx] > gMaxOverLoadCount && abs(load) > 250) {
      SerialBT.printf("LOAD LIMIT! Id %d Load: %d > 250 for %d times \n", id, load, gMaxOverLoadCount);
      auto s = sign(load);
      auto pos = fc.getPos(id);
      if (DEBUG) {
        SerialBT.printf("Id %d s: %d\n", id, s);
        SerialBT.printf("Id %d pos: %d\n", id, pos);
      }
      fc.moveFinger(pos + s * 10, id, 2000, 50);
      if (DEBUG) {
        SerialBT.printf("Id %d Moving to Pos %d \n", id, pos + s * 10);
      }
      delay(200);
      load = fc.getLoad(id);
    }
  }
}

void limitTemp() {

  for (int i = 0; i <= 4; i++) {
    int id = fc.getMotorIdByVectorIndex(static_cast<FingersController::VectorIdx>(i));
    auto temp = fc.getTemper(id);
    if (temp > 65) {
      fc.enableTorque(id, false);
      g_motor_enabled[i] = false;
      SerialBT.printf("Motor %d OVERHEAT! => Motor disabled! \n", id);
    } else if (!g_motor_enabled[i] && temp < 60) {
      fc.enableTorque(id, true);
      g_motor_enabled[i] = true;
      SerialBT.printf("Motor %d Cooled Down => Motor enabled! \n", id);
    }
  }
}

void processStringCmd(const String& cmd) {
  FingersController::GraspType value = FingersController::getGraspTypeByString(cmd);
  if (value < FingersController::GraspType::_MAX) {
    g_new_grasp_type = value;
    if (BLUETOOTH) SerialBT.println("Grasp by Serial Cmd");
  } else {  // Not a Grasp command..
    if (cmd == "INSTALL\r\n") {
      gFsmState = FSM::TendonInstall;
    } else if (cmd == "TEST\r\n") {
      gFsmState = FSM::Testing;
    } else if (cmd == "CONTROL\r\n") {
      gFsmState = FSM::Control;
    } else if (cmd.substring(0, 3) == "LIM") {  // Comando esempio: LIM6645
      g_load_limit[0] = cmd.substring(3, 4).toInt() * 111;
      g_load_limit[1] = cmd.substring(4, 5).toInt() * 111;
      g_load_limit[2] = cmd.substring(5, 6).toInt() * 111;
      g_load_limit[3] = cmd.substring(6, 7).toInt() * 111;
    } else if (cmd.substring(0, 5) == "INFO ") {
      int id = cmd.substring(5, cmd.lastIndexOf('\r')).toInt();
      fc.printFeedback(id);
    } else if (cmd.substring(0, 5) == "MOVE ") {
      int id = cmd.substring(5, 6).toInt();
      int pos = cmd.substring(6, cmd.lastIndexOf('\r')).toInt();
      fc.moveFinger(pos, id, 400, 2000);
    } else if (cmd.substring(0, 3) == "RT ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveFinger(factor, FingersController::VectorIdx::ThumbRot);
    } else if (cmd.substring(0, 3) == "FI ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveFinger(factor, FingersController::VectorIdx::Index);
    } else if (cmd.substring(0, 3) == "FM ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveFinger(factor, FingersController::VectorIdx::Middle);
    } else if (cmd.substring(0, 3) == "FR ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveFinger(factor, FingersController::VectorIdx::Ring);
    } else if (cmd.substring(0, 3) == "FT ") {
      int factor = cmd.substring(3, cmd.lastIndexOf('\r')).toInt();
      fc.moveFinger(factor, FingersController::VectorIdx::Thumb);
    } else if (cmd.substring(0, 11) == "CALICENTER ") {
      int id = cmd.substring(11, 12).toInt();
      fc.calibrateCenterOfRange(id);
    } else if (cmd.substring(0, 10) == "CALITENDON") {
      fc.performTendonCalibration();
    } else {
      g_newClosurePercent = cmd.toInt();
      if (BLUETOOTH) SerialBT.printf("ClosurePercent by Serial Cmd: %d\n", g_newClosurePercent);
    }
  }
}

void prepareGrasp() {

  if (g_grasp_type < FingersController::GraspType::_FINGER_CTRL_SEPARATOR) {
    fc.prepareGrasp(g_grasp_type);
  }
  g_preparation = false;
}

// TESTING FSM
enum class TestFSM {
  OPEN,
  CLOSE
};

TestFSM gTestFsmState = TestFSM::OPEN;
int gFsmTimer = millis();
int gTestCounter = -1;
void test() {
  const int limits[4] = { 330, 330, 330, 300 };
  switch (gTestFsmState) {

    case TestFSM::OPEN:
      fc.action(FingersController::GraspType::POWER, 0, limits);
      delay(2000);
      gTestFsmState = TestFSM::CLOSE;
      break;
    case TestFSM::CLOSE:
      fc.action(FingersController::GraspType::POWER, 100, limits);
      delay(2000);
      gTestFsmState = TestFSM::OPEN;
      gTestCounter++;
      break;
  }
  int tI = fc.getTemper(fc.getMotorIdByVectorIndex(FingersController::VectorIdx::Index));
  int tM = fc.getTemper(fc.getMotorIdByVectorIndex(FingersController::VectorIdx::Middle));
  int tR = fc.getTemper(fc.getMotorIdByVectorIndex(FingersController::VectorIdx::Ring));
  int tT = fc.getTemper(fc.getMotorIdByVectorIndex(FingersController::VectorIdx::Thumb));
  int tTR = fc.getTemper(fc.getMotorIdByVectorIndex(FingersController::VectorIdx::ThumbRot));
  SerialBT.printf("Test cycles: %d \t Temper: I:%d \t M:%d \t R:%d \t T:%d \t TR:%d \t \n", gTestCounter, tI, tM, tR, tT, tTR);
}

void loop() {

  if (BLUETOOTH && SerialBT.available()) {
    auto t1 = millis();
    auto cmd = SerialBT.readString();
    auto t2 = millis();
    SerialBT.printf("dt1: %d ms\n", t2 - t1);
    SerialBT.printf("string: %s \n", cmd);
    processStringCmd(cmd);
  }

  if (gFsmState == FSM::Control) {
    if (g_new_grasp_type != g_grasp_type) {
      auto full_cmd = FingersController::getGraspStringByType(g_new_grasp_type);
      auto reduced_cmd = full_cmd.substring(0, full_cmd.length() - 2);
      String msg = reduced_cmd + " GRASP ENABLED";

      if (BLUETOOTH) SerialBT.println(msg);

      g_preparation = true;
      g_closure = false;
      g_grasp_type = g_new_grasp_type;
    }

    if (g_newClosurePercent != g_closurePercent) {

      g_preparation = false;
      g_closure = true;
      g_closurePercent = g_newClosurePercent;
    }

    if (g_preparation) {
      prepareGrasp();
    }

    if (g_closure) {
      fc.action(g_grasp_type, g_closurePercent, g_load_limit);
      g_closure = false;
    }

    if (SHOW_FEEDBACK) {
      fc.printFeedback(INDEX_FLEX_MOTOR);
      fc.printFeedback(MIDDLE_FLEX_MOTOR);
      fc.printFeedback(RING_LITTLE_FLEX_MOTOR);
      fc.printFeedback(THUMB_FLEX_MOTOR);
      fc.printFeedback(THUMB_ROT_MOTOR);
    }


  } else if (gFsmState == FSM::TendonInstall) {

    SerialBT.println("Tendon Install..");
    fc.tendonInstallation();
    SerialBT.println("Please install tendons.");

  } else if (gFsmState == FSM::Testing) {
    test();
  } else if (gFsmState == FSM::DoNothing) {
    delay(10);
  }

  delay(1);

  limitLoad();
  limitTemp();
}