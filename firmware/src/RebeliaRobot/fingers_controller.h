
/*
Rebelia-Hand-Firmware is the control software for the Rebelia Hand, an Active Prosthetic Hand device (see https://www.robotgarage.org).

The Copyright Notice
Copyright (C)  2023 Vittorio Lumare

The License Notices
    This file is part of Rebelia-Hand-Firmware.

    Rebelia-Hand-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

    Rebelia-Hand-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with Rebelia-Hand-Firmware. If not, see <https://www.gnu.org/licenses/>. 
*/

#include <stdexcept>
#include <SCServo.h>
#include "BluetoothSerial.h"

#include <vector>
#include <functional>
using namespace std::placeholders;

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

static const bool DEBUG = false;

static int limit(const int val, const int a, const int b) {

  if (a <= b) {
    return constrain(val, a, b);
  } else
    return constrain(val, b, a);
}

class FingersController {
public:

  enum VectorIdx {
    Index = 0,
    Middle = 1,
    Ring = 2,
    Thumb = 3,
    ThumbRot = 4,
    Factor = 5
  };

  enum GraspType {
    POWER = 0,
    // HANDLE,
    POWERSMALL,
    MONKEY,
    PINCH,
    RELAX,
    POWERTOOL,
    _FINGER_CTRL_SEPARATOR,
    INDEX,
    MIDDLE,
    RING,
    THUMB,
    THUMB_ROT,
    _MAX
  };

  enum ClosingRotDir : int {
    CW = 1,
    CCW = -1
  };

public:
  static GraspType getGraspTypeByString(const String& cmd) {
    if (cmd == "POWER\r\n") {
      return FingersController::GraspType::POWER;
    }
    // else if (cmd == "HANDLE\r\n") {
    //   return FingersController::GraspType::HANDLE;
    // }
    else if (cmd == "POWERSMALL\r\n") {
      return FingersController::GraspType::POWERSMALL;
    } else if (cmd == "MONKEY\r\n") {
      return FingersController::GraspType::MONKEY;
    } else if (cmd == "PINCH\r\n") {
      return FingersController::GraspType::PINCH;
    } else if (cmd == "RELAX\r\n") {
      return FingersController::GraspType::RELAX;
    } else if (cmd == "INDEX\r\n") {
      return FingersController::GraspType::INDEX;
    } else if (cmd == "MIDDLE\r\n") {
      return FingersController::GraspType::MIDDLE;
    } else if (cmd == "RING\r\n") {
      return FingersController::GraspType::RING;
    } else if (cmd == "THUMB\r\n") {
      return FingersController::GraspType::THUMB;
    } else if (cmd == "THUMB_ROT\r\n") {
      return FingersController::GraspType::THUMB_ROT;
    }

    return FingersController::GraspType::_MAX;
  }

  static String getGraspStringByType(FingersController::GraspType type) {

    String text;

    switch (type) {
      case FingersController::GraspType::POWER:
        text = "POWER\r\n";
        break;
      // case FingersController::GraspType::HANDLE:
      //   text = "HANDLE\r\n";
      //   break;
      case FingersController::GraspType::POWERSMALL:
        text = "POWERSMALL\r\n";
        break;
      case FingersController::GraspType::MONKEY:
        text = "MONKEY\r\n";
        break;
      case FingersController::GraspType::PINCH:
        text = "PINCH\r\n";
        break;
      case FingersController::GraspType::RELAX:
        text = "RELAX\r\n";
        break;
      case FingersController::GraspType::INDEX:
        text = "INDEX\r\n";
        break;
      case FingersController::GraspType::MIDDLE:
        text = "MIDDLE\r\n";
        break;
      case FingersController::GraspType::RING:
        text = "RING\r\n";
        break;
      case FingersController::GraspType::THUMB:
        text = "THUMB\r\n";
        break;
      case FingersController::GraspType::THUMB_ROT:
        text = "THUMB_ROT\r\n";
        break;
      case FingersController::GraspType::_MAX:
        text = "INVALID_COMMAND\r\n";
        break;
    }

    return text;
  }

  void enableTorque(int ID, bool enable) {
    st.EnableTorque(ID, enable);
  }

public:
  FingersController() {
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    st.pSerial = &Serial1;

    // Init dynamic arrays
    power_matrix = (int**)calloc(ANY_MATRIX_ROWS, sizeof(int*));
    // handle_matrix = (int**)calloc(ANY_MATRIX_ROWS, sizeof(int*));
    powersmall_matrix = (int**)calloc(ANY_MATRIX_ROWS, sizeof(int*));
    monkey_matrix = (int**)calloc(ANY_MATRIX_ROWS, sizeof(int*));
    pinch_matrix = (int**)calloc(ANY_MATRIX_ROWS, sizeof(int*));
    relax_matrix = (int**)calloc(ANY_MATRIX_ROWS, sizeof(int*));
    powertool_matrix = (int**)calloc(ANY_MATRIX_ROWS, sizeof(int*));
    for (int i = 0; i < ANY_MATRIX_ROWS; i++) {
      power_matrix[i] = (int*)calloc(POWER_MATRIX_COLS, sizeof(int));
      // handle_matrix[i] = (int*)calloc(HANDLE_MATRIX_COLS, sizeof(int));
      powersmall_matrix[i] = (int*)calloc(POWERSMALL_MATRIX_COLS, sizeof(int));
      monkey_matrix[i] = (int*)calloc(MONKEY_MATRIX_COLS, sizeof(int));
      pinch_matrix[i] = (int*)calloc(PINCH_MATRIX_COLS, sizeof(int));
      relax_matrix[i] = (int*)calloc(RELAX_MATRIX_COLS, sizeof(int));
      powertool_matrix[i] = (int*)calloc(POWERTOOL_MATRIX_COLS, sizeof(int));
    }

    // Fill dynamic arrays
    for (int i = 0; i <= POWER_MATRIX_COLS; i++) {
      power_matrix[VectorIdx::Index][i] = POWER_INDEX[i];
      power_matrix[VectorIdx::Middle][i] = POWER_MIDDLE[i];
      power_matrix[VectorIdx::Ring][i] = POWER_RING[i];
      power_matrix[VectorIdx::Thumb][i] = POWER_THUMB[i];
      power_matrix[VectorIdx::ThumbRot][i] = POWER_THUMB_ROT[i];
      power_matrix[VectorIdx::Factor][i] = POWER_FACTOR[i];
    }

    // Fill dynamic arrays
    // for (int i = 0; i <= HANDLE_MATRIX_COLS; i++) {
    //   handle_matrix[VectorIdx::Index][i] = HANDLE_INDEX[i];
    //   handle_matrix[VectorIdx::Middle][i] = HANDLE_MIDDLE[i];
    //   handle_matrix[VectorIdx::Ring][i] = HANDLE_RING[i];
    //   handle_matrix[VectorIdx::Thumb][i] = HANDLE_THUMB[i];
    //   handle_matrix[VectorIdx::ThumbRot][i] = HANDLE_THUMB_ROT[i];
    //   handle_matrix[VectorIdx::Factor][i] = HANDLE_FACTOR[i];
    // }

    for (int i = 0; i <= POWERSMALL_MATRIX_COLS; i++) {
      powersmall_matrix[VectorIdx::Index][i] = POWERSMALL_INDEX[i];
      powersmall_matrix[VectorIdx::Middle][i] = POWERSMALL_MIDDLE[i];
      powersmall_matrix[VectorIdx::Ring][i] = POWERSMALL_RING[i];
      powersmall_matrix[VectorIdx::Thumb][i] = POWERSMALL_THUMB[i];
      powersmall_matrix[VectorIdx::ThumbRot][i] = POWERSMALL_THUMB_ROT[i];
      powersmall_matrix[VectorIdx::Factor][i] = POWERSMALL_FACTOR[i];
    }
    for (int i = 0; i <= MONKEY_MATRIX_COLS; i++) {
      monkey_matrix[VectorIdx::Index][i] = MONKEY_INDEX[i];
      monkey_matrix[VectorIdx::Middle][i] = MONKEY_MIDDLE[i];
      monkey_matrix[VectorIdx::Ring][i] = MONKEY_RING[i];
      monkey_matrix[VectorIdx::Thumb][i] = MONKEY_THUMB[i];
      monkey_matrix[VectorIdx::ThumbRot][i] = MONKEY_THUMB_ROT[i];
      monkey_matrix[VectorIdx::Factor][i] = MONKEY_FACTOR[i];
    }
    for (int i = 0; i <= PINCH_MATRIX_COLS; i++) {
      pinch_matrix[VectorIdx::Index][i] = PINCH_INDEX[i];
      pinch_matrix[VectorIdx::Middle][i] = PINCH_MIDDLE[i];
      pinch_matrix[VectorIdx::Ring][i] = PINCH_RING[i];
      pinch_matrix[VectorIdx::Thumb][i] = PINCH_THUMB[i];
      pinch_matrix[VectorIdx::ThumbRot][i] = PINCH_THUMB_ROT[i];
      pinch_matrix[VectorIdx::Factor][i] = PINCH_FACTOR[i];
    }
    for (int i = 0; i <= RELAX_MATRIX_COLS; i++) {
      relax_matrix[VectorIdx::Index][i] = RELAX_INDEX[i];
      relax_matrix[VectorIdx::Middle][i] = RELAX_MIDDLE[i];
      relax_matrix[VectorIdx::Ring][i] = RELAX_RING[i];
      relax_matrix[VectorIdx::Thumb][i] = RELAX_THUMB[i];
      relax_matrix[VectorIdx::ThumbRot][i] = RELAX_THUMB_ROT[i];
      relax_matrix[VectorIdx::Factor][i] = RELAX_FACTOR[i];
    }
    for (int i = 0; i <= POWERTOOL_MATRIX_COLS; i++) {
      powertool_matrix[VectorIdx::Index][i] = POWERTOOL_INDEX[i];
      powertool_matrix[VectorIdx::Middle][i] = POWERTOOL_MIDDLE[i];
      powertool_matrix[VectorIdx::Ring][i] = POWERTOOL_RING[i];
      powertool_matrix[VectorIdx::Thumb][i] = POWERTOOL_THUMB[i];
      powertool_matrix[VectorIdx::ThumbRot][i] = POWERTOOL_THUMB_ROT[i];
      powertool_matrix[VectorIdx::Factor][i] = POWERTOOL_FACTOR[i];
    }
  }

  ~FingersController() {
    free(power_matrix);
    // free(handle_matrix);
    free(powersmall_matrix);
    free(monkey_matrix);
    free(pinch_matrix);
    free(relax_matrix);
    free(powertool_matrix);
  }

public:

  void tendonInstallation() {
    for (int idx = VectorIdx::Index; idx <= VectorIdx::Thumb; idx++) {
      int id = getMotorIdByVectorIndex((VectorIdx)idx);
      moveFinger(MOTORS_POS_TENDON_INSTALLATION[idx], id, 2000, 50);
    }
  }

  void calibrateCenterOfRange(int motor_id) {
    int result = st.CalibrationOfs(motor_id);
    if (result == 0) {
      SerialBT.println("Successfully calibrated center of range! ");
    } else {
      SerialBT.println("Failed center of range calibration! ");
    }
  }

  void performTendonCalibration() {
    const int ABS_LOAD_LIMIT_FOR_OPENING[4] = { 220, 220, 400, 200 };  //220, 220, 400, 200
    const int ABS_LOAD_LIMIT_FOR_CLOSING[4] = { 400, 400, 400, 400 };
    const int START_POS_FOR_OPENING[4] = { 1000, 1000, 1000, 1200 };
    const int START_POS_FOR_CLOSING[4] = { 2500, 2500, 2500, 1200 };
    const int END_POS_FOR_OPENING[4] = { 50, 50, 50, 50 };
    const int END_POS_FOR_CLOSING[4] = { 4045, 4045, 4045, 4045 };
    const int LOOP_DELAY = 50;
    const int ABS_INCR = 10;

    auto fp_opening = std::bind(&FingersController::decrementAndCheckLoad, this, _1, _2, _3, _4, _5, _6);
    auto fp_closing = std::bind(&FingersController::incrementAndCheckLoad, this, _1, _2, _3, _4, _5, _6);


    const int VELOCITY = 2000;
    const int ACCELERATION = 50;

    moveFinger(START_POS_FOR_OPENING[0], INDEX_FLEX_MOTOR, VELOCITY, ACCELERATION);
    moveFinger(START_POS_FOR_OPENING[1], MIDDLE_FLEX_MOTOR, VELOCITY, ACCELERATION);
    moveFinger(START_POS_FOR_OPENING[2], RING_LITTLE_FLEX_MOTOR, VELOCITY, ACCELERATION);
    moveFinger(START_POS_FOR_OPENING[3], THUMB_FLEX_MOTOR, VELOCITY, ACCELERATION);
    delay(10);
    moveFinger(MOTORS_POS_RANGE[VectorIdx::ThumbRot][0], THUMB_ROT_MOTOR, VELOCITY, ACCELERATION);
    delay(1000);
    moveFingersWhileChecking(fp_opening, ABS_LOAD_LIMIT_FOR_OPENING, POS_RANGE_MIN_IDX, START_POS_FOR_OPENING, END_POS_FOR_OPENING, LOOP_DELAY, ABS_INCR);

    moveFinger(START_POS_FOR_CLOSING[0], INDEX_FLEX_MOTOR, VELOCITY, ACCELERATION);
    moveFinger(START_POS_FOR_CLOSING[1], MIDDLE_FLEX_MOTOR, VELOCITY, ACCELERATION);
    moveFinger(START_POS_FOR_CLOSING[2], RING_LITTLE_FLEX_MOTOR, VELOCITY, ACCELERATION);
    moveFinger(START_POS_FOR_CLOSING[3], THUMB_FLEX_MOTOR, VELOCITY, ACCELERATION);
    delay(10);
    moveFinger(MOTORS_POS_RANGE[VectorIdx::ThumbRot][0], THUMB_ROT_MOTOR, VELOCITY, ACCELERATION);
    delay(1000);
    moveFingersWhileChecking(fp_closing, ABS_LOAD_LIMIT_FOR_CLOSING, POS_RANGE_MAX_IDX, START_POS_FOR_CLOSING, END_POS_FOR_CLOSING, LOOP_DELAY, ABS_INCR);

    auto degreeRange = [](int min, int max) {
      return (int)(((double)abs(max - min) / 4096.0) * 360.0);
    };

    SerialBT.println("All tendons calibrated!");
    auto iMin = getAbsolutePos(VectorIdx::Index, 0);
    auto iMax = getAbsolutePos(VectorIdx::Index, 100);
    auto iDeg = degreeRange(iMin, iMax);
    auto mMin = getAbsolutePos(VectorIdx::Middle, 0);
    auto mMax = getAbsolutePos(VectorIdx::Middle, 100);
    auto mDeg = degreeRange(mMin, mMax);
    auto rMin = getAbsolutePos(VectorIdx::Ring, 0);
    auto rMax = getAbsolutePos(VectorIdx::Ring, 100);
    auto rDeg = degreeRange(rMin, rMax);
    auto tMin = getAbsolutePos(VectorIdx::Thumb, 0);
    auto tMax = getAbsolutePos(VectorIdx::Thumb, 100);
    auto tDeg = degreeRange(tMin, tMax);

    SerialBT.printf("I(%d,%d)[%d deg] M(%d,%d)[%d deg] RL(%d,%d)[%d deg] T (%d,%d)[%d deg]\n", iMin, iMax, iDeg, mMin, mMax, mDeg, rMin, rMax, rDeg, tMin, tMax, tDeg);
  }

  void closeUntilLoadIsReached(const GraspType grasp_type, const int end_frame, const int load_limit[4]) {

    static int cur_grasp_type = GraspType::MONKEY;
    static int cur_frame = 0;

    if (grasp_type != cur_grasp_type) {
      cur_grasp_type = grasp_type;
      cur_frame = 0;
    }

    GraspData grasp_data;
    getGraspData(grasp_type, grasp_data);

    SerialBT.println("grasp_data:");
    for (int i = 0; i < grasp_data.rows; i++) {
      for (int j = 0; j < grasp_data.cols; j++) {
        SerialBT.printf("%d\t", grasp_data.matrix[i][j]);
      }
      SerialBT.println();
    }
    SerialBT.println();
    SerialBT.println();

    int START_POS[4];
    int END_POS[4];
    const int LOOP_DELAY = 10;
    const int ABS_INCR = 20;  // MAX 20 !

    if (cur_frame < end_frame) {
      for (int frame = cur_frame; frame < min(end_frame, grasp_data.cols - 1); frame++) {
        SerialBT.printf("increasing frame: %d\n", frame);
        SerialBT.print("END_POS:");
        getPosFromTrajectory(grasp_type, frame, START_POS);
        getPosFromTrajectory(grasp_type, frame + 1, END_POS);
        for (int i = 0; i < 4; i++) {
          SerialBT.printf("%d\t", END_POS[i]);
        }
        SerialBT.println();
        auto fp_closing = std::bind(&FingersController::incrementAndCheckLoad, this, _1, _2, _3, _4, _5, _6);
        moveFingersWhileChecking(fp_closing, load_limit, -1, START_POS, END_POS, LOOP_DELAY, ABS_INCR);
      }
    } else if (end_frame < cur_frame) {
      for (int frame = cur_frame; frame > max(end_frame, 0); frame--) {
        SerialBT.printf("decreasing frame: %d\n", frame);
        SerialBT.print("END_POS:");
        getPosFromTrajectory(grasp_type, frame, START_POS);
        getPosFromTrajectory(grasp_type, frame - 1, END_POS);
        for (int i = 0; i < 4; i++) {
          SerialBT.printf("%d\t", END_POS[i]);
        }
        SerialBT.println();
        auto fp_closing = std::bind(&FingersController::decrementAndCheckLoad, this, _1, _2, _3, _4, _5, _6);
        moveFingersWhileChecking(fp_closing, load_limit, -1, START_POS, END_POS, LOOP_DELAY, ABS_INCR);
      }
    }
    cur_frame = end_frame;
  }

  void
  moveFingersWhileChecking(std::function<bool(int, int, int, int, int&, int)> checkFunction, const int abs_load_limit[4], const int destination_index, const int start_pos[4], const int end_pos[4], int loop_delay, int abs_increment) {

    bool allLimitsOfEndposReached = false;
    bool limit_or_endpos_reached[4] = { false, false, false, false };
    int pos[4] = { start_pos[0], start_pos[1], start_pos[2], start_pos[3] };

    // DEBUG
    if (DEBUG) { SerialBT.print("abs_load_limit: "); }
    for (int i = 0; i < 4; i++) {
      SerialBT.print(abs_load_limit[i]);
      SerialBT.print(",");
    }
    SerialBT.println();

    while (!allLimitsOfEndposReached) {

      if (!limit_or_endpos_reached[VectorIdx::Index]) {
        moveFinger(pos[VectorIdx::Index], INDEX_FLEX_MOTOR, 2000, 50);
      }
      if (!limit_or_endpos_reached[VectorIdx::Middle]) {
        moveFinger(pos[VectorIdx::Middle], MIDDLE_FLEX_MOTOR, 2000, 50);
      }

      if (!limit_or_endpos_reached[VectorIdx::Ring]) {
        moveFinger(pos[VectorIdx::Ring], RING_LITTLE_FLEX_MOTOR, 2000, 50);
      }

      if (!limit_or_endpos_reached[VectorIdx::Thumb]) {
        moveFinger(pos[VectorIdx::Thumb], THUMB_FLEX_MOTOR, 1500, 50);
      }

      // Wait all fingers moved;
      delay(loop_delay);
      if (!limit_or_endpos_reached[VectorIdx::Index]) {
        limit_or_endpos_reached[VectorIdx::Index] = checkFunction(INDEX_FLEX_MOTOR, ClosingRotDir::CW,
                                                                  abs_load_limit[VectorIdx::Index], abs_increment, pos[VectorIdx::Index], end_pos[VectorIdx::Index]);
        if (limit_or_endpos_reached[VectorIdx::Index]) {
          if (DEBUG) { SerialBT.println("Index: limit or pos reached."); }
        }
      }
      if (!limit_or_endpos_reached[VectorIdx::Middle]) {
        limit_or_endpos_reached[VectorIdx::Middle] = checkFunction(MIDDLE_FLEX_MOTOR, ClosingRotDir::CW,
                                                                   abs_load_limit[VectorIdx::Middle], abs_increment, pos[VectorIdx::Middle], end_pos[VectorIdx::Middle]);
        if (limit_or_endpos_reached[VectorIdx::Middle]) {
          if (DEBUG) { SerialBT.println("Middle: limit or pos reached."); }
        }
      }

      if (!limit_or_endpos_reached[VectorIdx::Ring]) {
        limit_or_endpos_reached[VectorIdx::Ring] = checkFunction(RING_LITTLE_FLEX_MOTOR, ClosingRotDir::CW,
                                                                 abs_load_limit[VectorIdx::Ring], abs_increment, pos[VectorIdx::Ring], end_pos[VectorIdx::Ring]);
        if (limit_or_endpos_reached[VectorIdx::Ring]) {
          if (DEBUG) { SerialBT.println("Ring: limit or pos reached."); }
        }
      }

      if (!limit_or_endpos_reached[VectorIdx::Thumb]) {
        limit_or_endpos_reached[VectorIdx::Thumb] = checkFunction(THUMB_FLEX_MOTOR, ClosingRotDir::CW,
                                                                  abs_load_limit[VectorIdx::Thumb], abs_increment, pos[VectorIdx::Thumb], end_pos[VectorIdx::Thumb]);
        if (limit_or_endpos_reached[VectorIdx::Thumb]) {
          if (DEBUG) { SerialBT.println("Thumb: limit or pos reached."); }
        }
      }

      allLimitsOfEndposReached = limit_or_endpos_reached[VectorIdx::Index] && limit_or_endpos_reached[VectorIdx::Middle] && limit_or_endpos_reached[VectorIdx::Ring] && limit_or_endpos_reached[VectorIdx::Thumb];

      if (DEBUG && allLimitsOfEndposReached) {
        if (DEBUG) { SerialBT.println("All load limits reached."); }
      }
    }

    // ONLY FOR TENDON CALIBRATION!
    if (destination_index >= 0) {  // if <0 skip
      SerialBT.println("Setting values..");
      MOTORS_POS_RANGE[VectorIdx::Index][destination_index] = readPos(INDEX_FLEX_MOTOR);
      MOTORS_POS_RANGE[VectorIdx::Middle][destination_index] = readPos(MIDDLE_FLEX_MOTOR);
      MOTORS_POS_RANGE[VectorIdx::Ring][destination_index] = readPos(RING_LITTLE_FLEX_MOTOR);
      MOTORS_POS_RANGE[VectorIdx::Thumb][destination_index] = readPos(THUMB_FLEX_MOTOR);
      SerialBT.println("..values are set.");
    }
  }

  bool decrementAndCheckLoad(int id, int closing_rot_dir, int abs_load_limit, int abs_increment, int& pos, const int end_pos) {
    int load = readLoad(id);
    if (DEBUG || true) { SerialBT.printf("I: %d L: %d P: %d\n", id, load, pos); }
    pos -= closing_rot_dir * abs_increment;
    return !(((closing_rot_dir == ClosingRotDir::CW && load > -closing_rot_dir * abs_load_limit) || (closing_rot_dir == ClosingRotDir::CCW && load < -closing_rot_dir * abs_load_limit))
             && (pos > end_pos && pos < 4045));
  }

  bool incrementAndCheckLoad(int id, int closing_rot_dir, int abs_load_limit, int abs_increment, int& pos, const int end_pos) {
    // Half increment for thumb so it's slower and doesn't collide with index
    // abs_increment = (id == THUMB_FLEX_MOTOR) ? abs_increment / 3 : abs_increment;

    int load = readLoad(id);
    if (DEBUG || true) { SerialBT.printf("I: %d L: %d P: %d\n", id, load, pos); }
    pos += closing_rot_dir * abs_increment;
    return !(
      (
        (closing_rot_dir == ClosingRotDir::CW && load > -closing_rot_dir * abs_load_limit) || (closing_rot_dir == ClosingRotDir::CCW && load < -closing_rot_dir * abs_load_limit))
      && (pos > 50 && pos < end_pos));
  }

  void calibrateOpenSingleFinger(int id, int closing_rot_dir, int abs_load_limit, int abs_increment) {
    int pos = 2048;
    moveFinger(pos, id, 2000, 50);
    delay(1000);
    int load = 0;
    while (((closing_rot_dir == ClosingRotDir::CW && load < -closing_rot_dir * abs_load_limit) || (closing_rot_dir == ClosingRotDir::CCW && load > -closing_rot_dir * abs_load_limit)) && (pos > 50 && pos < 4000)) {
      moveFinger(pos, id, 2000, 50);
      delay(50);
      load = readLoad(id);
      SerialBT.printf("L: %d P: %d\n", load, pos);
      pos += closing_rot_dir * abs_increment;
    }

    SerialBT.printf("Motor ID %d Calibrated !\n", id);
  }

  void calibrateCloseingleFinger(int id, int closing_rot_dir, int abs_load_limit, int abs_increment) {
    int pos = 2048;
    moveFinger(pos, id, 2000, 50);
    delay(1000);
    int load = 0;
    while (((closing_rot_dir == ClosingRotDir::CW && load > closing_rot_dir * abs_load_limit) || (closing_rot_dir == ClosingRotDir::CCW && load < closing_rot_dir * abs_load_limit)) && (pos > 50 && pos < 4000)) {
      moveFinger(pos, id, 2000, 50);
      delay(50);
      load = readLoad(id);
      SerialBT.printf("L: %d P: %d\n", load, pos);
      pos -= closing_rot_dir * abs_increment;
    }

    SerialBT.printf("Motor ID %d Calibrated !\n", id);
  }

  int getAbsolutePos(int finger_idx, int factor) {
    int absolute_pos_value = map(factor, 0, 100, MOTORS_POS_RANGE[finger_idx][POS_RANGE_MIN_IDX], MOTORS_POS_RANGE[finger_idx][POS_RANGE_MAX_IDX]);
    absolute_pos_value = limit(absolute_pos_value, MOTORS_POS_RANGE[finger_idx][POS_RANGE_MIN_IDX], MOTORS_POS_RANGE[finger_idx][POS_RANGE_MAX_IDX]);
    return absolute_pos_value;
  }

  GraspType getGraspTypeByThumbRot() {
    auto pos = getPos(THUMB_ROT_MOTOR);
    if (abs(pos - POWER_THUMB_ROT[0]) < 50) {
      return FingersController::GraspType::POWER;
    } else if (abs(pos - POWERSMALL_THUMB_ROT[0]) < 100) {
      return FingersController::GraspType::POWERSMALL;
    } else if (abs(pos - MONKEY_THUMB_ROT[0]) < 100) {
      return FingersController::GraspType::MONKEY;
    } else if (abs(pos - PINCH_THUMB_ROT[0]) < 100) {
      return FingersController::GraspType::PINCH;
    } else if (abs(pos - RELAX_THUMB_ROT[0]) < 100) {
      return FingersController::GraspType::RELAX;
    }

    return FingersController::GraspType::_MAX;
  }

  void prepareMonkeyGrasp() {
    SerialBT.println("Preparing Monkey Grasp");

    int finger_pos[4];
    for (int i = VectorIdx::Index; i <= VectorIdx::Thumb; i++) {
      finger_pos[i] = getAbsolutePos(i, monkey_matrix[i][0]);
    }

    for (int i = VectorIdx::Index; i <= VectorIdx::Thumb; i++) {
      st.RegWritePosEx(FINGER_MOTOR_ID[i], finger_pos[i], 3400, 50);
    }
    st.RegWriteAction();
    delay(10);

    int thumb_rot_pos = getAbsolutePos(VectorIdx::ThumbRot, monkey_matrix[VectorIdx::ThumbRot][0]);
    st.RegWritePosEx(THUMB_ROT_MOTOR, thumb_rot_pos, 3400, 50);
    st.RegWriteAction();
    delay(10);

    SerialBT.println("Monkey Grasp Prepared");
  }

  void monkeyGrasp(const int gesture_factor) {
    byte ID[] = { INDEX_FLEX_MOTOR, MIDDLE_FLEX_MOTOR, RING_LITTLE_FLEX_MOTOR, THUMB_FLEX_MOTOR };
    u16 Speed[] = { 4000, 4000, 4000, 4000 };
    byte ACC[] = { 200, 200, 200, 200 };

    int flex_joint_pos_value[4];

    for (size_t i = 0; i < MONKEY_MATRIX_COLS - 1; i++) {
      auto p1 = MONKEY_FACTOR[i];
      auto p2 = MONKEY_FACTOR[i + 1];
      if (gesture_factor >= p1 && gesture_factor <= p2) {

        for (int finger_idx = VectorIdx::Index; finger_idx <= VectorIdx::Thumb; finger_idx++) {  // iterate fingers
          int fingerPos1 = getAbsolutePos(finger_idx, monkey_matrix[finger_idx][i]);
          int fingerPos2 = getAbsolutePos(finger_idx, monkey_matrix[finger_idx][i + 1]);
          flex_joint_pos_value[finger_idx] = map(gesture_factor, p1, p2, fingerPos1, fingerPos2);
          flex_joint_pos_value[finger_idx] = limit(flex_joint_pos_value[finger_idx], fingerPos1, fingerPos2);
        }
        break;
      }
    }


    s16 Position[4] = { flex_joint_pos_value[VectorIdx::Index], flex_joint_pos_value[VectorIdx::Middle], flex_joint_pos_value[VectorIdx::Ring], flex_joint_pos_value[VectorIdx::Thumb] };

    //auto t1 = micros();
    st.SyncWritePosEx(ID, 4, Position, Speed, ACC);  //servo(ID1/ID2) speed=3400, acc=50, move to position=3000.
                                                     //auto t2 = micros();
                                                     //auto dt = (t2 - t1) * 1.0 / 1000.0;
                                                     //SerialBT.printf("SyncWritePosEx [ms]:  %f \n", dt);
  }

  void getPosFromTrajectory(GraspType grasp_type, int frame, int initial_factor[4]) {

    GraspData grasp_data;
    getGraspData(grasp_type, grasp_data);

    for (int i = VectorIdx::Index; i <= VectorIdx::Thumb; i++) {
      initial_factor[i] = getAbsolutePos(i, grasp_data.matrix[i][frame]);
    }
  }

  void prepareGrasp(GraspType grasp_type) {

    SerialBT.println("Preparing Grasp");
    int initial_factor[4];
    getPosFromTrajectory(grasp_type, 0, initial_factor);

    for (int i = VectorIdx::Index; i <= VectorIdx::Thumb; i++) {
      st.RegWritePosEx(FINGER_MOTOR_ID[i], initial_factor[i], 3400, 50);
    }
    st.RegWriteAction();
    delay(10);


    GraspData grasp_data;
    getGraspData(grasp_type, grasp_data);

    int thumb_rot_pos = getAbsolutePos(VectorIdx::ThumbRot, grasp_data.matrix[VectorIdx::ThumbRot][0]);
    st.RegWritePosEx(THUMB_ROT_MOTOR, thumb_rot_pos, 3400, 50);
    st.RegWriteAction();
    delay(10);

    SerialBT.println("Grasp Prepared");
  }

  void grasp(const GraspType grasp_type, const int gesture_factor) {
    GraspData grasp_data;
    getGraspData(grasp_type, grasp_data);

    byte ID[] = { INDEX_FLEX_MOTOR, MIDDLE_FLEX_MOTOR, RING_LITTLE_FLEX_MOTOR, THUMB_FLEX_MOTOR };
    u16 Speed[] = { 2000, 2000, 2000, 2000 };
    byte ACC[] = { 200, 200, 200, 200 };

    int flex_joint_pos_value[4];
    bool position_set = false;
    for (size_t i = 0; i < grasp_data.cols - 1; i++) {
      auto p1 = monkey_matrix[VectorIdx::Factor][i];
      auto p2 = monkey_matrix[VectorIdx::Factor][i + 1];
      if (gesture_factor >= p1 && gesture_factor <= p2) {

        for (int finger_idx = VectorIdx::Index; finger_idx <= VectorIdx::Thumb; finger_idx++) {  // iterate fingers
          int fingerPos1 = getAbsolutePos(finger_idx, grasp_data.matrix[finger_idx][i]);
          int fingerPos2 = getAbsolutePos(finger_idx, grasp_data.matrix[finger_idx][i + 1]);
          flex_joint_pos_value[finger_idx] = map(gesture_factor, p1, p2, fingerPos1, fingerPos2);
          flex_joint_pos_value[finger_idx] = limit(flex_joint_pos_value[finger_idx], fingerPos1, fingerPos2);
        }
        position_set = true;
        break;
      }
    }

    if (position_set) {
      s16 Position[4] = { flex_joint_pos_value[VectorIdx::Index], flex_joint_pos_value[VectorIdx::Middle], flex_joint_pos_value[VectorIdx::Ring], flex_joint_pos_value[VectorIdx::Thumb] };
      st.SyncWritePosEx(ID, 4, Position, Speed, ACC);
    }
  }

  int getMotorIdByVectorIndex(const VectorIdx idx) {
    return idx + 1;
  }


  void moveFinger(const int pos, const int motor_id, const int speed, const int accel) {
    byte ID[] = { motor_id };
    u16 Speed[] = { speed };
    byte ACC[] = { accel };
    s16 Position[] = { pos };
    st.SyncWritePosEx(ID, 1, Position, Speed, ACC);
  }

  void moveAllFingersToMiddlePosition() {
    for (int id = 1; id <= 5; id++) {
      SerialBT.printf("Moving to middle position id: ..", id);
      moveFinger(2048, id, 100, 200);
      SerialBT.println(".. moved!");
    }
  }

  int moveFingerSync(const int factor, VectorIdx idx) {

    byte ID = getMotorIdByVectorIndex(idx);
    u16 Speed = 4000;
    byte ACC = 200;

    auto joint_pos_value = map(factor, 0, 100, MOTORS_POS_RANGE[idx][0], MOTORS_POS_RANGE[idx][1]);
    joint_pos_value = limit(joint_pos_value, MOTORS_POS_RANGE[idx][0], MOTORS_POS_RANGE[idx][1]);

    auto err = st.WritePosEx(idx, joint_pos_value, Speed, ACC);

    return err;
  }

  void moveFinger(const int factor, VectorIdx idx) {

    byte ID[] = { getMotorIdByVectorIndex(idx) };
    u16 Speed[] = { 4000 };
    byte ACC[] = { 200 };

    auto joint_pos_value = map(factor, 0, 100, MOTORS_POS_RANGE[idx][0], MOTORS_POS_RANGE[idx][1]);
    joint_pos_value = limit(joint_pos_value, MOTORS_POS_RANGE[idx][0], MOTORS_POS_RANGE[idx][1]);
    s16 Position[] = { joint_pos_value };
    st.SyncWritePosEx(ID, 1, Position, Speed, ACC);
  }

  int readPos(const int motorId) {
    return st.ReadPos(motorId);
  }

  bool isMoving(const int motorId) {
    bool moving = st.ReadMove(motorId) == 1 ? true : false;
    return moving;
  }

  void testCalib(const int motor_id, const int offset = 0) {
    st.WritePosEx(motor_id, 2048 + offset, 3400, 50);
  }

  void showLoad() {
    SerialBT.print("LOAD : TR: ");
    SerialBT.print(st.ReadLoad(THUMB_ROT_MOTOR));
    SerialBT.print("  TF: ");
    SerialBT.print(st.ReadLoad(THUMB_FLEX_MOTOR));
    SerialBT.print("  IF: ");
    SerialBT.print(st.ReadLoad(INDEX_FLEX_MOTOR));
    SerialBT.print("  MF: ");
    SerialBT.print(st.ReadLoad(MIDDLE_FLEX_MOTOR));
    SerialBT.print("  RLF: ");
    SerialBT.println(st.ReadLoad(RING_LITTLE_FLEX_MOTOR));
  }

  int readLoad(const int motorID) {
    return st.ReadLoad(motorID);
  }

  void action(const FingersController::GraspType grasp_type, const int factor, const int load_limit[4]) {
    switch (grasp_type) {
      case FingersController::GraspType::POWER:
      // case FingersController::GraspType::HANDLE:
      case FingersController::GraspType::POWERSMALL:
      case FingersController::GraspType::MONKEY:
      case FingersController::GraspType::PINCH:
      case FingersController::GraspType::RELAX:
        closeUntilLoadIsReached(grasp_type, factor, load_limit);
        break;
      case FingersController::GraspType::INDEX:
        moveFinger(factor, VectorIdx::Index);
        break;
      case FingersController::GraspType::MIDDLE:
        moveFinger(factor, VectorIdx::Middle);
        break;
      case FingersController::GraspType::RING:
        moveFinger(factor, VectorIdx::Ring);
        break;
      case FingersController::GraspType::THUMB:
        moveFinger(factor, VectorIdx::Thumb);
        break;
      case FingersController::GraspType::THUMB_ROT:
        moveFinger(factor, VectorIdx::ThumbRot);
        break;
    }
  }


  void pingTest(const int motorID) {
    int ID = st.Ping(motorID);
    if (ID != -1) {
      Serial.print("Servo ID:");
      Serial.println(ID, DEC);
      SerialBT.printf("Servo ID: %d \n", ID);
      delay(100);
    } else {
      Serial.println("Ping servo ID error!");
      SerialBT.printf("Ping servo ID error!\n");
      delay(100);
    }
  }

  int getPos(const int motorID) {
    return st.ReadPos(motorID);
  }

  int getLoad(const int motorID) {
    return st.ReadLoad(motorID);
  }

  int getTemper(const int motorID) {
    return st.ReadTemper(motorID);
  }

  void printFeedback(const int id) {
    int Pos;
    int Speed;
    int Load;
    int Voltage;
    int Temper;
    int Move;
    int Current;
    if (st.FeedBack(id) != -1) {
      Pos = st.ReadPos(id);
      Speed = st.ReadSpeed(id);
      Load = st.ReadLoad(id);
      Voltage = st.ReadVoltage(id);
      Temper = st.ReadTemper(id);
      Move = st.ReadMove(id);
      Current = st.ReadCurrent(id);
      SerialBT.printf("ID:%d, P:%d, L:%d, V:%d, T:%d, M:%d, C:%d\n", id, Pos, Load, Voltage, Temper, Move, Current);
      delay(10);
    } else {
      SerialBT.println("FeedBack err");
      delay(500);
    }
  }

  void changeID(const int currentId, const int newId) {
    st.unLockEprom(currentId);                   // Unlock EPROM-SAFE
    st.writeByte(currentId, SMS_STS_ID, newId);  // Change ID
    st.LockEprom(newId);                         // EPROM-SAFE is locked

    int ID = st.Ping(newId);
    if (ID == newId) {
      SerialBT.println("New ID successfully set!");
    } else {
      SerialBT.println("FAILED to set new ID!");
    }
  }



  void setTrajectoryData(int tj[6][2], VectorIdx vidx, int frame, int value) {
    tj[vidx][frame] = value;
  }

public:
  // Monkey Vector
  struct GraspData {
    int** matrix;
    int rows;
    int cols;
  };

  const void getGraspData(GraspType grasp_type, GraspData& data) {
    data.rows = ANY_MATRIX_ROWS;

    switch (grasp_type) {
      case GraspType::POWER:
        data.matrix = power_matrix;
        data.cols = POWER_MATRIX_COLS;
        break;
      // case GraspType::HANDLE:
      //   data.matrix = handle_matrix;
      //   data.cols = HANDLE_MATRIX_COLS;
      //   break;
      case GraspType::POWERSMALL:
        data.matrix = powersmall_matrix;
        data.cols = POWERSMALL_MATRIX_COLS;
        break;
      case GraspType::MONKEY:
        data.matrix = monkey_matrix;
        data.cols = MONKEY_MATRIX_COLS;
        break;
      case GraspType::PINCH:
        data.matrix = pinch_matrix;
        data.cols = PINCH_MATRIX_COLS;
        break;
      case GraspType::RELAX:
        data.matrix = relax_matrix;
        data.cols = RELAX_MATRIX_COLS;
        break;
      case GraspType::POWERTOOL:
        data.matrix = powertool_matrix;
        data.cols = POWERTOOL_MATRIX_COLS;
        break;
    }
  }

public:
  void setMaxTorque(const int ID, const int max_torque) {
    st.EnableTorque(ID, 0);  // Disable torque
    st.writeWord(ID, SMS_STS_TORQUE_LIMIT_L, max_torque);
    st.EnableTorque(ID, 1);  // Enable with limit
  }

public:
  // Motors
  static const int INDEX_FLEX_MOTOR = 1;
  static const int MIDDLE_FLEX_MOTOR = 2;
  static const int RING_LITTLE_FLEX_MOTOR = 3;
  static const int THUMB_FLEX_MOTOR = 4;
  static const int THUMB_ROT_MOTOR = 5;
  constexpr static int FINGER_MOTOR_ID[5] = { INDEX_FLEX_MOTOR, MIDDLE_FLEX_MOTOR, RING_LITTLE_FLEX_MOTOR, THUMB_FLEX_MOTOR, THUMB_ROT_MOTOR };

public:

  const int POS_RANGE_MIN_IDX = 0;
  const int POS_RANGE_MAX_IDX = 1;
  int MOTORS_POS_RANGE[5][2] = { { 2048, 2048 }, { 2048, 2048 }, { 2048, 2048 }, { 2048, 2048 }, { 1080, 1980 } };
  int MOTORS_POS_TENDON_INSTALLATION[4] = { 512, 512, 512, 512 };  //3583

  // Positions
  // Convention: from open state to close state
  static const int ANY_MATRIX_ROWS = 6;

  static const int POWER_MATRIX_COLS = 3;
  int** power_matrix;
  int POWER_INDEX[POWER_MATRIX_COLS] = { 0, 0, 100 };       // FINGER FACTOR [0,100]
  int POWER_MIDDLE[POWER_MATRIX_COLS] = { 0, 0, 100 };      // FINGER FACTOR [0,100]
  int POWER_RING[POWER_MATRIX_COLS] = { 0, 0, 100 };        // FINGER FACTOR [0,100]
  int POWER_THUMB[POWER_MATRIX_COLS] = { 0, 60, 100 };      // FINGER FACTOR [0,100]
  int POWER_THUMB_ROT[POWER_MATRIX_COLS] = { 90, 90, 90 };  // FINGER FACTOR [0,100]
  int POWER_FACTOR[POWER_MATRIX_COLS] = { 0, 30, 100 };     // MAIN GESTURE FACTOR [0,100]

  // static const int HANDLE_MATRIX_COLS = 4;
  // int** handle_matrix;
  // int HANDLE_INDEX[HANDLE_MATRIX_COLS] = { 0, 30, 100, 100 };
  // int HANDLE_MIDDLE[HANDLE_MATRIX_COLS] = { 0, 30, 100, 100};
  // int HANDLE_RING[HANDLE_MATRIX_COLS] = { 0, 30, 100, 100};
  // int HANDLE_THUMB[HANDLE_MATRIX_COLS] = { 0, 0, 0, 100};
  // int HANDLE_THUMB_ROT[HANDLE_MATRIX_COLS] = { 90, 90, 90, 90 };
  // int HANDLE_FACTOR[HANDLE_MATRIX_COLS] = { 0, 33, 66, 100 };

  static const int POWERSMALL_MATRIX_COLS = 3;
  int** powersmall_matrix;
  int POWERSMALL_INDEX[POWERSMALL_MATRIX_COLS] = { 0, 100, 100 };
  int POWERSMALL_MIDDLE[POWERSMALL_MATRIX_COLS] = { 0, 100, 100 };
  int POWERSMALL_RING[POWERSMALL_MATRIX_COLS] = { 0, 100, 100 };
  int POWERSMALL_THUMB[POWERSMALL_MATRIX_COLS] = { 0, 1, 100 };
  int POWERSMALL_THUMB_ROT[POWERSMALL_MATRIX_COLS] = { 60, 60, 60 };
  int POWERSMALL_FACTOR[POWERSMALL_MATRIX_COLS] = { 0, 70, 100 };

  static const int MONKEY_MATRIX_COLS = 4;
  int** monkey_matrix;
  int MONKEY_INDEX[MONKEY_MATRIX_COLS] = { 0, 50, 100, 100 };
  int MONKEY_MIDDLE[MONKEY_MATRIX_COLS] = { 0, 50, 100, 100 };
  int MONKEY_RING[MONKEY_MATRIX_COLS] = { 0, 50, 100, 100 };
  int MONKEY_THUMB[MONKEY_MATRIX_COLS] = { 0, 50, 50, 100 };
  int MONKEY_THUMB_ROT[MONKEY_MATRIX_COLS] = { 0, 0, 0, 0 };
  int MONKEY_FACTOR[MONKEY_MATRIX_COLS] = { 0, 70, 70, 100 };

  static const int PINCH_MATRIX_COLS = 7;
  int** pinch_matrix;
  int PINCH_INDEX[PINCH_MATRIX_COLS] = { 0, 30, 40, 50, 60, 65, 70 };  //50 -> 70
  int PINCH_MIDDLE[PINCH_MATRIX_COLS] = { 0, 0, 0, 0, 0, 0, 0 };
  int PINCH_RING[PINCH_MATRIX_COLS] = { 0, 0, 0, 0, 0, 0, 0 };
  int PINCH_THUMB[PINCH_MATRIX_COLS] = { 70, 70, 70, 70, 70, 75, 80 };
  int PINCH_THUMB_ROT[PINCH_MATRIX_COLS] = { 40, 40, 40, 40, 40, 40, 40 };  //40
  int PINCH_FACTOR[PINCH_MATRIX_COLS] = { 0, 10, 20, 40, 60, 80, 100 };

  static const int RELAX_MATRIX_COLS = 3;
  int** relax_matrix;
  int RELAX_INDEX[RELAX_MATRIX_COLS] = { 20, 20, 20 };
  int RELAX_MIDDLE[RELAX_MATRIX_COLS] = { 30, 30, 30 };
  int RELAX_RING[RELAX_MATRIX_COLS] = { 30, 40, 40 };
  int RELAX_THUMB[RELAX_MATRIX_COLS] = { 80, 80, 80 };
  int RELAX_THUMB_ROT[RELAX_MATRIX_COLS] = { 50, 50, 50 };
  int RELAX_FACTOR[RELAX_MATRIX_COLS] = { 0, 100, 100 };

  static const int POWERTOOL_MATRIX_COLS = 3;
  int** powertool_matrix;
  int POWERTOOL_INDEX[POWERTOOL_MATRIX_COLS] = { 0, 0, 100 };
  int POWERTOOL_MIDDLE[POWERTOOL_MATRIX_COLS] = { 0, 100, 100 };
  int POWERTOOL_RING[POWERTOOL_MATRIX_COLS] = { 0, 100, 100 };
  int POWERTOOL_THUMB[POWERTOOL_MATRIX_COLS] = { 0, 100, 100 };
  int POWERTOOL_THUMB_ROT[POWERTOOL_MATRIX_COLS] = { 20, 20, 20 };
  int POWERTOOL_FACTOR[POWERTOOL_MATRIX_COLS] = { 0, 50, 100 };

  const bool DEBUG_LOG = true;

private:  // load control
  const int DYN_LOAD_POS_ERROR_THR = 50;
  const int MIN_CLOSURE_FOR_LOAD_CTRL = 30;

private:
  SMS_STS st;
  BluetoothSerial SerialBT;
};
