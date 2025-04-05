/*
  Rebelia-Hand-Firmware is the control software for the Rebelia Hand

  Website: https://www.robotgarage.org
  Author: Vittorio Lumare
  Email: vittorio.lumar@robotgarage.org 

  GNU GPL License

  Copyright (c) 2025 Vittorio Lumare

  
  This file is part of Rebelia-Hand-Firmware.

  Rebelia-Hand-Firmware is free software: you can redistribute it and/or 
  modify it under the terms of the GNU General Public License as published 
  by the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version.

  Rebelia-Hand-Firmware is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
  
  See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along 
  with Rebelia-Hand-Firmware. If not, see <https://www.gnu.org/licenses/>. 
*/


#include "fingers_controller.h"
#include "fft_manager.h"
#include "BluetoothSerial.h"

/* Check if Bluetooth configurations are enabled in the SDK */
/* If not, then you have to recompile the SDK */
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define DEBUG_LOG true

static const bool BLUETOOTH = true; // Control by Bluetooth Serial
static const bool PING_TEST = false; // Test Motor Connection
static const bool EMG_TEST = false; // Test EMG sensors


FingersController fc;
BluetoothSerial SerialBT;

void setup() {

  Serial.begin(115200);
  Serial.println("Bluetooth Started! Ready to pair...");

  if (BLUETOOTH) SerialBT.begin();

  delay(1000);

  // !!!  *** FINGER SERVO CALIBRATION *** !!!
  // fc.calibrate(FingersController::THUMB_FLEX_MOTOR);
  // fc.testCalib(FingersController::THUMB_FLEX_MOTOR);
  // while (true){
  //     Serial.println("Calibrated");
  //     SerialBT.println("Calibrated");
  //     delay(1000);
  // }

  while (PING_TEST) {
    // fc.pingTest(FingersController::INDEX_FLEX_MOTOR);
    fc.pingTest(FingersController::MIDDLE_FLEX_MOTOR);
    // fc.pingTest(FingersController::RING_LITTLE_FLEX_MOTOR);
    // fc.pingTest(FingersController::THUMB_FLEX_MOTOR);
    // fc.pingTest(FingersController::THUMB_ROT_MOTOR);
  }

  // EMG init
  {
    pinMode(EMG_OPEN_PIN, INPUT);
    pinMode(EMG_CLOSE_PIN, INPUT);
  }
  while (EMG_TEST) {
    // EMG INPUT PINS TEST
    int o = analogRead(EMG_OPEN_PIN);
    int c = analogRead(EMG_CLOSE_PIN);
    Serial.print("o: ");
    Serial.print(o);
    Serial.print("\t c: ");
    Serial.println(c);
    SerialBT.printf("o: %d \t c: %d \n", o, c);
    delay(100);
  }
}

// Configuration
int RELAX_TIME_SEC_THR = 60;

// State Machine
bool g_preparation = true;
bool g_closure = false;
int g_closurePercent = 0;
int g_last_time_when_closure_was_not_zero = millis();
String g_emg_sequence;
FingersController::GraspType g_grasp_type = FingersController::GraspType::RELAX;
FingersController::GraspType g_new_grasp_type = g_grasp_type;

const std::vector<String> double_open_patterns = { "NONON", "NOONOON", "NONOON", "NOONON", "NOOOON", "NOOON", "NOON" };

FFTManager g_fft_man;

void updateGraspTypeByThumbPos() {
  if (g_closurePercent == 0 || g_grasp_type == FingersController::GraspType::RELAX) {
    fc.enableTorque(FingersController::THUMB_ROT_MOTOR, false);
    if (BLUETOOTH) SerialBT.println("Thumb Rot Torque Disabled");

    FingersController::GraspType value = fc.getGraspTypeByThumbRot();
    if (value < FingersController::GraspType::_MAX) {
      g_new_grasp_type = value;
    } else {
      if (BLUETOOTH) SerialBT.println("GraspType not recognized..");
    }
  }
}

void updateGraspTypeByEMG() {
  for (const auto pattern : double_open_patterns) {
    if (g_emg_sequence.indexOf(pattern) != -1) {
      g_new_grasp_type = static_cast<FingersController::GraspType>(static_cast<int>(g_grasp_type) + 1);
      if (g_new_grasp_type == FingersController::GraspType::_MAX) {
        g_new_grasp_type = static_cast<FingersController::GraspType>(0);  //reset to first one
      }
      g_emg_sequence.clear();
      break;
    }
  }
}

void processStringCmd(const String& cmd) {
  FingersController::GraspType value = FingersController::getGraspTypeByString(cmd);
  if (value < FingersController::GraspType::_MAX) {
    g_new_grasp_type = value;
    if (BLUETOOTH) SerialBT.println("Grasp by Serial Cmd");
  } else {
    g_closurePercent = cmd.toInt();
    if (BLUETOOTH) SerialBT.printf("ClosurePercent by Serial Cmd: %d\n", g_closurePercent);
  }
}

void prepareGrasp() {
  Serial.println("Preparing..");
  fc.prepareGrasp(g_grasp_type);
  Serial.println("..prepared");
  g_preparation = false;
  g_closure = true;
}

void updateClosureByEMG() {

  auto oo = g_fft_man.process(EMG_OPEN_PIN);
  auto cc = g_fft_man.process(EMG_CLOSE_PIN);

  if (DEBUG_LOG) {
    // Serial.print("oo:");
    // Serial.print(oo);
    // Serial.print("\t");
    // Serial.print("cc:");
    // Serial.print(cc);
    // Serial.println("\t");
    //SerialBT.printf("oo %f\n", oo);
    //SerialBT.printf("cc %f\n", cc);
  }

  char emg_cmd;

  // EMG Decision Maker
  if (oo > EMG_OPEN_THR)  //&& cc < C_THR)
  {
    if (DEBUG_LOG) Serial.println("O"); // Open
    if (BLUETOOTH) SerialBT.println("O"); // Open

    emg_cmd = 'O';
  }
  if (oo < EMG_OPEN_THR && cc > EMG_CLOSE_THR) {
    if (DEBUG_LOG) Serial.println("C"); // Close
    if (BLUETOOTH) SerialBT.println("C"); // Close
    emg_cmd = 'C';
  }
  if (oo < EMG_OPEN_THR && cc < EMG_CLOSE_THR) {
    if (DEBUG_LOG) Serial.println("N"); // No action
    if (BLUETOOTH) SerialBT.println("N"); // No action
    emg_cmd = 'N';
  }

  if (g_closurePercent == 0) {
    // add EMG command to circular buffer
    g_emg_sequence = String(emg_cmd) + g_emg_sequence;
    if (g_emg_sequence.length() > 10) {
      g_emg_sequence = g_emg_sequence.substring(0, 10);
    }
  } else {
    g_emg_sequence.clear();
  }

  if (emg_cmd == 'C') { // Close
    g_closurePercent += 5;
  }
  if (emg_cmd == 'O') { // Open
    g_closurePercent -= 5;
  }
  g_closurePercent = constrain(g_closurePercent, 0, 100);

  if (BLUETOOTH) {
    SerialBT.printf("Closure Percent by EMG: %d %\n", g_closurePercent);
  }
  if (DEBUG_LOG) Serial.print("Closure Percent by EMG: ");
  if (DEBUG_LOG) Serial.println(g_closurePercent);

  if (g_closurePercent != 0) {
    g_last_time_when_closure_was_not_zero = millis();
  }
}

int showSerialCmd(const String& cmd) {
  SerialBT.println("CMD RECEIVED: ");
  for (auto c : cmd) {
    SerialBT.println(static_cast<int>(c));
  }
}

const bool EMG_CONTROL = true;
const bool FINGERS_TEST = false;
const bool CALIBRATION = false;
void loop() {

  if (EMG_CONTROL) {
    if (millis() - g_last_time_when_closure_was_not_zero > RELAX_TIME_SEC_THR * 1000) {
      g_new_grasp_type = FingersController::GraspType::RELAX;
      g_closurePercent = 0;  // In case it was changed for any reason by some function
      if (BLUETOOTH) SerialBT.println("Going to RELAX..");
    }
  }

  //updateGraspTypeByEMG();

  if (g_new_grasp_type != g_grasp_type) {
    auto full_cmd = FingersController::getGraspStringByType(g_new_grasp_type);
    auto reduced_cmd = full_cmd.substring(0, full_cmd.length() - 2);
    String msg = reduced_cmd + " GRASP ENABLED";

    if (BLUETOOTH) SerialBT.println(msg);

    g_preparation = true;
    g_closure = false;
    g_grasp_type = g_new_grasp_type;
  }

  if (EMG_CONTROL) {

    updateClosureByEMG();

    if (BLUETOOTH && SerialBT.available()) {
      auto cmd = SerialBT.readString();
      processStringCmd(cmd);
    } else {
      updateGraspTypeByThumbPos();
    }


    if (g_preparation) {
      prepareGrasp();
    }

    if (g_closure) {
      if (g_closurePercent > 0) {
        fc.enableTorque(FingersController::THUMB_ROT_MOTOR, true);
        if (BLUETOOTH) SerialBT.println("Thumb Rot Torque Enabled");
      }
      const bool sync = true;
      const int load_limit = 360;
      fc.grasp(g_grasp_type, g_closurePercent);
    }
  }

  if (CALIBRATION) {

    if (BLUETOOTH && SerialBT.available()) {
      auto cmd = SerialBT.readString();
      processStringCmd(cmd);
    } else {
      updateGraspTypeByThumbPos();
    }

    if (g_preparation) {
      prepareGrasp();
    }

    if (g_closure) {
      if (g_closurePercent > 0) {
        fc.enableTorque(FingersController::THUMB_ROT_MOTOR, true);
        if (BLUETOOTH) SerialBT.println("Thumb Rot Torque Enabled");
      }
      fc.grasp(g_grasp_type, g_closurePercent);
    }
  }

  delay(1);
}
