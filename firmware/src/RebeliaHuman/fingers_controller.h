#include <stdexcept>
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

#include <SCServo.h>
#include "BluetoothSerial.h"

#include <vector>

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

static int limit(const int val, const int a, const int b) {

  if (a <= b) {
    return constrain(val, a, b);
  } else
    return constrain(val, b, a);
}

class FingersController {
public:
  enum GraspType {
    POWER = 0,
    POWERSMALL,
    MONKEY,
    PINCH,
    RELAX,
    _MAX
  };

public:
  static GraspType getGraspTypeByString(const String& cmd) {
    if (cmd == "POWER\r\n") {
      return FingersController::GraspType::POWER;
    } else if (cmd == "POWERSMALL\r\n") {
      return FingersController::GraspType::POWERSMALL;
    } else if (cmd == "MONKEY\r\n") {
      return FingersController::GraspType::MONKEY;
    } else if (cmd == "PINCH\r\n") {
      return FingersController::GraspType::PINCH;
    } else if (cmd == "RELAX\r\n") {
      return FingersController::GraspType::RELAX;
    }

    return FingersController::GraspType::_MAX;
  }



  static String getGraspStringByType(FingersController::GraspType type) {

    String text;

    switch (type) {
      case FingersController::GraspType::POWER:
        text = "POWER\r\n";
        break;
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

    //      SerialBT.begin();
  }

  // EXAMPLE: st.RegWritePosEx(5, 2150, 3400, 50);//servo(ID1) speed=3400，acc=50，move to position=4095.
public:

  void calibrate(int motor_id) {
    st.CalibrationOfs(motor_id);
  }

  GraspType getGraspTypeByThumbRot() {

    auto pos = getPos(THUMB_ROT_MOTOR);
    if (abs(pos - POWER_THUMB_ROT.front()) < 50) {
      return FingersController::GraspType::POWER;
    } else if (abs(pos - POWERSMALL_THUMB_ROT.front()) < 100) {
      return FingersController::GraspType::POWERSMALL;
    } else if (abs(pos - MONKEY_THUMB_ROT.front()) < 100) {
      return FingersController::GraspType::MONKEY;
    } else if (abs(pos - PINCH_THUMB_ROT.front()) < 100) {
      return FingersController::GraspType::PINCH;
    } else if (abs(pos - RELAX_THUMB_ROT.front()) < 100) {
      return FingersController::GraspType::RELAX;
    }

    return FingersController::GraspType::_MAX;
  }

  void preparePowerGrasp() {

    st.RegWritePosEx(THUMB_FLEX_MOTOR, POWER_THUMB.front(), 3400, 50);
    st.RegWriteAction();
    delay(500);

    st.RegWritePosEx(INDEX_FLEX_MOTOR, POWER_INDEX.front(), 3400, 50);
    st.RegWritePosEx(MIDDLE_FLEX_MOTOR, POWER_MIDDLE.front(), 3400, 50);
    st.RegWritePosEx(RING_LITTLE_FLEX_MOTOR, POWER_RING.front(), 3400, 50);
    st.RegWriteAction();
    delay(500);

    st.RegWritePosEx(THUMB_ROT_MOTOR, POWER_THUMB_ROT.front(), 3400, 50);
    st.RegWriteAction();
    delay(500);
  }

  void powerGrasp(const int factor) {

    byte ID[] = { INDEX_FLEX_MOTOR, MIDDLE_FLEX_MOTOR, RING_LITTLE_FLEX_MOTOR, THUMB_FLEX_MOTOR };
    u16 Speed[] = { 3400, 3400, 3400, 3400 };
    byte ACC[] = { 50, 50, 50, 50 };

    auto index_flex_joint_pos_value = map(factor, 0, 100, POWER_INDEX.front(), POWER_INDEX.back());
    index_flex_joint_pos_value = limit(index_flex_joint_pos_value, POWER_INDEX.front(), POWER_INDEX.back());

    auto middle_flex_joint_pos_value = map(factor, 0, 100, POWER_MIDDLE.front(), POWER_MIDDLE.back());
    middle_flex_joint_pos_value = limit(middle_flex_joint_pos_value, POWER_MIDDLE.front(), POWER_MIDDLE.back());

    auto ring_little_flex_joint_pos_value = map(factor, 0, 100, POWER_RING.front(), POWER_RING.back());
    ring_little_flex_joint_pos_value = limit(ring_little_flex_joint_pos_value, POWER_RING.front(), POWER_RING.back());

    auto thumb_flex_joint_pos_value = map(factor, 0, 30, POWER_THUMB.front(), POWER_THUMB.back());
    thumb_flex_joint_pos_value = limit(thumb_flex_joint_pos_value, POWER_THUMB.front(), POWER_THUMB.back());

    s16 Position[] = { index_flex_joint_pos_value, middle_flex_joint_pos_value, ring_little_flex_joint_pos_value,
                       thumb_flex_joint_pos_value };

    //auto t1 = micros();
    st.SyncWritePosEx(ID, 4, Position, Speed, ACC);  //servo(ID1/ID2) speed=3400, acc=50, move to position=3000.
    //auto t2 = micros();
    //auto dt = (t2 - t1) * 1.0 / 1000.0;
    //SerialBT.printf("SyncWritePosEx [ms]:  %f \n", dt);
    //SerialBT.printf("I%d \tM%d \tR%d \tT%d \n", index_flex_joint_pos_value, middle_flex_joint_pos_value, ring_little_flex_joint_pos_value, thumb_flex_joint_pos_value);
  }

  void preparePowerSmallGrasp() {

    st.RegWritePosEx(THUMB_FLEX_MOTOR, POWERSMALL_THUMB.front(), 3400, 50);
    st.RegWriteAction();
    delay(500);

    st.RegWritePosEx(INDEX_FLEX_MOTOR, POWERSMALL_INDEX.front(), 3400, 50);
    st.RegWritePosEx(MIDDLE_FLEX_MOTOR, POWERSMALL_MIDDLE.front(), 3400, 50);
    st.RegWritePosEx(RING_LITTLE_FLEX_MOTOR, POWERSMALL_RING.front(), 3400, 50);
    st.RegWriteAction();
    delay(500);

    st.RegWritePosEx(THUMB_ROT_MOTOR, POWERSMALL_THUMB_ROT.front(), 3400, 50);
    st.RegWriteAction();
    delay(500);
  }

  void powerSmallGrasp(const int factor) {

    byte ID[] = { INDEX_FLEX_MOTOR, MIDDLE_FLEX_MOTOR, RING_LITTLE_FLEX_MOTOR, THUMB_FLEX_MOTOR };
    u16 Speed[] = { 3400, 3400, 3400, 3400 };
    byte ACC[] = { 50, 50, 50, 50 };

    auto index_flex_joint_pos_value = map(factor, 0, 70, POWERSMALL_INDEX.front(), POWERSMALL_INDEX.back());
    index_flex_joint_pos_value = limit(index_flex_joint_pos_value, POWERSMALL_INDEX.front(), POWERSMALL_INDEX.back());

    auto middle_flex_joint_pos_value = map(factor, 0, 70, POWERSMALL_MIDDLE.front(), POWERSMALL_MIDDLE.back());
    middle_flex_joint_pos_value = limit(middle_flex_joint_pos_value, POWERSMALL_MIDDLE.front(), POWERSMALL_MIDDLE.back());

    auto ring_little_flex_joint_pos_value = map(factor, 0, 70, POWERSMALL_RING.front(), POWERSMALL_RING.back());
    ring_little_flex_joint_pos_value = limit(ring_little_flex_joint_pos_value, POWERSMALL_RING.front(), POWERSMALL_RING.back());

    auto thumb_flex_joint_pos_value = map(factor, 70, 100, POWERSMALL_THUMB.front(), POWERSMALL_THUMB.back());
    thumb_flex_joint_pos_value = limit(thumb_flex_joint_pos_value, POWERSMALL_THUMB.front(), POWERSMALL_THUMB.back());

    s16 Position[] = { index_flex_joint_pos_value, middle_flex_joint_pos_value, ring_little_flex_joint_pos_value,
                       thumb_flex_joint_pos_value };

    st.SyncWritePosEx(ID, 4, Position, Speed, ACC);
  }


  void powerSmallGraspSync(const int factor, const bool sync = false, const int load_limit = 1000) {

    static int prev_factor = factor;

    if (!sync && load_limit < 1000) {
      SerialBT.println("Load limit is possible only with sync enabled!");
      throw std::runtime_error("Load limit is possible only with sync enabled!");
    } else if (sync) {
      SerialBT.printf("Load limit: %d \n", load_limit);
    }

    int index_pos = st.ReadPos(INDEX_FLEX_MOTOR);
    int middle_pos = st.ReadPos(MIDDLE_FLEX_MOTOR);
    int ring_pos = st.ReadPos(RING_LITTLE_FLEX_MOTOR);
    int thumb_pos = st.ReadPos(THUMB_FLEX_MOTOR);

    static bool index_load_limited = false;
    static bool middle_load_limited = false;
    static bool ring_load_limited = false;
    static bool thumb_load_limited = false;
    if (factor < prev_factor) {
      index_load_limited = false;
      middle_load_limited = false;
      ring_load_limited = false;
      thumb_load_limited = false;
    }

    byte ID[] = { INDEX_FLEX_MOTOR, MIDDLE_FLEX_MOTOR, RING_LITTLE_FLEX_MOTOR, THUMB_FLEX_MOTOR };
    u16 Speed[] = { 3400, 3400, 3400, 3400 };
    byte ACC[] = { 50, 50, 50, 50 };

    auto index_flex_joint_pos_value = map(factor, 0, 70, POWERSMALL_INDEX.front(), POWERSMALL_INDEX.back());
    index_flex_joint_pos_value = limit(index_flex_joint_pos_value, POWERSMALL_INDEX.front(), POWERSMALL_INDEX.back());
    if (index_load_limited && index_flex_joint_pos_value > index_pos) index_flex_joint_pos_value = index_pos;
    //not working. if (index_flex_joint_pos_value < index_pos) index_load_limited = false;

    auto middle_flex_joint_pos_value = map(factor, 0, 70, POWERSMALL_MIDDLE.front(), POWERSMALL_MIDDLE.back());
    middle_flex_joint_pos_value = limit(middle_flex_joint_pos_value, POWERSMALL_MIDDLE.front(), POWERSMALL_MIDDLE.back());
    if (middle_load_limited && middle_flex_joint_pos_value < middle_pos) middle_flex_joint_pos_value = middle_pos;
    //not working. if (middle_flex_joint_pos_value > middle_pos) middle_load_limited = false;

    auto ring_little_flex_joint_pos_value = map(factor, 0, 70, POWERSMALL_RING.front(), POWERSMALL_RING.back());
    ring_little_flex_joint_pos_value = limit(ring_little_flex_joint_pos_value, POWERSMALL_RING.front(), POWERSMALL_RING.back());
    if (ring_load_limited && ring_little_flex_joint_pos_value < ring_pos) ring_little_flex_joint_pos_value = ring_pos;
    //not working. if (ring_little_flex_joint_pos_value > ring_pos) ring_load_limited = false;

    auto thumb_flex_joint_pos_value = map(factor, 70, 100, POWERSMALL_THUMB.front(), POWERSMALL_THUMB.back());
    thumb_flex_joint_pos_value = limit(thumb_flex_joint_pos_value, POWERSMALL_THUMB.front(), POWERSMALL_THUMB.back());
    if (thumb_load_limited && thumb_flex_joint_pos_value > thumb_pos) thumb_flex_joint_pos_value = thumb_pos;
    //not working. if (thumb_flex_joint_pos_value < thumb_pos) thumb_load_limited = false;

    s16 Position[] = { index_flex_joint_pos_value, middle_flex_joint_pos_value, ring_little_flex_joint_pos_value,
                       thumb_flex_joint_pos_value };

    st.SyncWritePosEx(ID, 4, Position, Speed, ACC);

    if (sync) {

      index_pos = st.ReadPos(INDEX_FLEX_MOTOR);
      middle_pos = st.ReadPos(MIDDLE_FLEX_MOTOR);
      ring_pos = st.ReadPos(RING_LITTLE_FLEX_MOTOR);
      thumb_pos = st.ReadPos(THUMB_FLEX_MOTOR);
      int index_err = abs(index_pos - index_flex_joint_pos_value);
      int middle_err = abs(middle_pos - middle_flex_joint_pos_value);
      int ring_err = abs(ring_pos - ring_little_flex_joint_pos_value);
      int thumb_err = abs(thumb_pos - thumb_flex_joint_pos_value);
      bool index_load_captured = false;
      bool middle_load_captured = false;
      bool ring_load_captured = false;
      bool thumb_load_captured = false;
      auto index_load = 0;
      auto middle_load = 0;
      auto ring_load = 0;
      auto thumb_load = 0;
      while (index_err > DYN_LOAD_POS_ERROR_THR || middle_err > DYN_LOAD_POS_ERROR_THR || ring_err > DYN_LOAD_POS_ERROR_THR || thumb_err > DYN_LOAD_POS_ERROR_THR) {
        index_pos = st.ReadPos(INDEX_FLEX_MOTOR);
        middle_pos = st.ReadPos(MIDDLE_FLEX_MOTOR);
        ring_pos = st.ReadPos(RING_LITTLE_FLEX_MOTOR);
        thumb_pos = st.ReadPos(THUMB_FLEX_MOTOR);
        index_err = abs(index_pos - index_flex_joint_pos_value);
        middle_err = abs(middle_pos - middle_flex_joint_pos_value);
        ring_err = abs(ring_pos - ring_little_flex_joint_pos_value);
        thumb_err = abs(thumb_pos - thumb_flex_joint_pos_value);

        if (!index_load_captured && index_err <= DYN_LOAD_POS_ERROR_THR) {
          index_load_captured = true;
          index_load = st.ReadLoad(INDEX_FLEX_MOTOR);
        }
        if (!middle_load_captured && middle_err <= DYN_LOAD_POS_ERROR_THR) {
          middle_load_captured = true;
          middle_load = st.ReadLoad(MIDDLE_FLEX_MOTOR);
        }
        if (!ring_load_captured && ring_err <= DYN_LOAD_POS_ERROR_THR) {
          ring_load_captured = true;
          ring_load = st.ReadLoad(RING_LITTLE_FLEX_MOTOR);
        }
        if (!thumb_load_captured && thumb_err <= DYN_LOAD_POS_ERROR_THR) {
          thumb_load_captured = true;
          thumb_load = st.ReadLoad(THUMB_FLEX_MOTOR);
        }

        // Debug
        SerialBT.printf("S I p:%d  %d  L:%d\n", index_pos, index_flex_joint_pos_value, index_load);
        // SerialBT.printf("S M p:%d  %d  L:%d\n", middle_pos, middle_flex_joint_pos_value, middle_load);
        // SerialBT.printf("S R p:%d  %d  L:%d\n", ring_pos, ring_little_flex_joint_pos_value, ring_load);
        // SerialBT.printf("S T p:%d  %d  L:%d\n", thumb_pos, thumb_flex_joint_pos_value, thumb_load);
      }

      // load limiter
      if (factor > MIN_CLOSURE_FOR_LOAD_CTRL) {

        index_load_limited += abs(index_load) > abs(load_limit);
        middle_load_limited += abs(middle_load) > abs(load_limit);
        ring_load_limited += abs(ring_load) > abs(load_limit);
        thumb_load_limited += abs(thumb_load) > abs(load_limit);

        // Debug
        SerialBT.printf("L I L:%d Lim:%d\n", index_load, index_load_limited);
        // SerialBT.printf("L M L:%d Lim:%d\n", middle_load, middle_load_limited);
        // SerialBT.printf("L R L:%d Lim:%d\n", ring_load, ring_load_limited);
        // SerialBT.printf("L T L:%d Lim:%d\n", thumb_load, thumb_load_limited);
      }
    }
    prev_factor = factor;
  }

  void preparePowerToolGrasp() {

    st.RegWritePosEx(THUMB_FLEX_MOTOR, POWERTOOL_THUMB.front(), 3400, 50);
    st.RegWriteAction();
    delay(500);

    st.RegWritePosEx(INDEX_FLEX_MOTOR, POWERTOOL_INDEX.front(), 3400, 50);
    st.RegWritePosEx(MIDDLE_FLEX_MOTOR, POWERTOOL_MIDDLE.front(), 3400, 50);
    st.RegWritePosEx(RING_LITTLE_FLEX_MOTOR, POWERTOOL_RING.front(), 3400, 50);
    st.RegWriteAction();
    delay(500);

    st.RegWritePosEx(THUMB_ROT_MOTOR, POWERTOOL_THUMB_ROT.front(), 3400, 50);
    st.RegWriteAction();
    delay(500);
  }

  void powerToolGrasp(const int factor) {

    byte ID[] = { INDEX_FLEX_MOTOR, MIDDLE_FLEX_MOTOR, RING_LITTLE_FLEX_MOTOR, THUMB_FLEX_MOTOR };
    u16 Speed[] = { 3400, 3400, 3400, 3400 };
    byte ACC[] = { 50, 50, 50, 50 };

    auto index_flex_joint_pos_value = map(factor, 30, 100, POWERTOOL_INDEX.front(), POWERTOOL_INDEX.back());
    index_flex_joint_pos_value = limit(index_flex_joint_pos_value, POWERTOOL_INDEX.front(), POWERTOOL_INDEX.back());

    auto middle_flex_joint_pos_value = map(factor, 0, 30, POWERTOOL_MIDDLE.front(), POWERTOOL_MIDDLE.back());
    middle_flex_joint_pos_value = limit(middle_flex_joint_pos_value, POWERTOOL_MIDDLE.front(), POWERTOOL_MIDDLE.back());

    auto ring_little_flex_joint_pos_value = map(factor, 0, 30, POWERTOOL_RING.front(), POWERTOOL_RING.back());
    ring_little_flex_joint_pos_value = limit(ring_little_flex_joint_pos_value, POWERTOOL_RING.front(), POWERTOOL_RING.back());

    auto thumb_flex_joint_pos_value = map(factor, 0, 30, POWERTOOL_THUMB.front(), POWERTOOL_THUMB.back());
    thumb_flex_joint_pos_value = limit(thumb_flex_joint_pos_value, POWERTOOL_THUMB.front(), POWERTOOL_THUMB.back());

    s16 Position[] = { index_flex_joint_pos_value, middle_flex_joint_pos_value, ring_little_flex_joint_pos_value,
                       thumb_flex_joint_pos_value };

    //auto t1 = micros();
    st.SyncWritePosEx(ID, 4, Position, Speed, ACC);  //servo(ID1/ID2) speed=3400, acc=50, move to position=3000.
    //auto t2 = micros();
    //auto dt = (t2 - t1) * 1.0 / 1000.0;
    //SerialBT.printf("SyncWritePosEx [ms]:  %f \n", dt);
  }

  void prepareRelax() {

    st.RegWritePosEx(THUMB_FLEX_MOTOR, RELAX_THUMB.front(), 6400, 50);
    st.RegWriteAction();
    delay(10);

    st.RegWritePosEx(INDEX_FLEX_MOTOR, RELAX_INDEX.front(), 6400, 50);
    st.RegWritePosEx(MIDDLE_FLEX_MOTOR, RELAX_MIDDLE.front(), 6400, 50);
    st.RegWritePosEx(RING_LITTLE_FLEX_MOTOR, RELAX_RING.front(), 6400, 50);
    st.RegWriteAction();
    delay(10);

    st.RegWritePosEx(THUMB_ROT_MOTOR, RELAX_THUMB_ROT.front(), 6400, 50);
    st.RegWriteAction();
    delay(10);
  }


  void relaxGrasp(const int factor) {
    byte ID[] = { INDEX_FLEX_MOTOR, MIDDLE_FLEX_MOTOR, RING_LITTLE_FLEX_MOTOR, THUMB_FLEX_MOTOR };
    u16 Speed[] = { 3400, 3400, 3400, 3400 };
    byte ACC[] = { 50, 50, 50, 50 };

    auto index_flex_joint_pos_value = map(factor, 0, 50, RELAX_INDEX.front(), RELAX_INDEX.back());
    index_flex_joint_pos_value = limit(index_flex_joint_pos_value, RELAX_INDEX.front(), RELAX_INDEX.back());

    auto middle_flex_joint_pos_value = map(factor, 0, 50, RELAX_MIDDLE.front(), RELAX_MIDDLE.back());
    middle_flex_joint_pos_value = limit(middle_flex_joint_pos_value, RELAX_MIDDLE.front(), RELAX_MIDDLE.back());

    auto ring_little_flex_joint_pos_value = map(factor, 0, 50, RELAX_RING.front(), RELAX_RING.back());
    ring_little_flex_joint_pos_value = limit(ring_little_flex_joint_pos_value, RELAX_RING.front(), RELAX_RING.back());

    auto thumb_flex_joint_pos_value = map(factor, 60, 100, RELAX_THUMB.front(), RELAX_THUMB.back());
    thumb_flex_joint_pos_value = limit(thumb_flex_joint_pos_value, RELAX_THUMB.front(), RELAX_THUMB.back());

    s16 Position[] = { index_flex_joint_pos_value, middle_flex_joint_pos_value, ring_little_flex_joint_pos_value, thumb_flex_joint_pos_value };

    auto t1 = micros();
    st.SyncWritePosEx(ID, 4, Position, Speed, ACC);  //servo(ID1/ID2) speed=3400, acc=50, move to position=3000.
    auto t2 = micros();
    auto dt = (t2 - t1) * 1.0 / 1000.0;
    //SerialBT.printf("SyncWritePosEx [ms]:  %f \n", dt);
  }


  void prepareMonkeyGrasp() {

    st.RegWritePosEx(THUMB_FLEX_MOTOR, MONKEY_THUMB.front(), 3400, 50);
    st.RegWriteAction();
    delay(10);

    st.RegWritePosEx(INDEX_FLEX_MOTOR, MONKEY_INDEX.front(), 3400, 50);
    st.RegWritePosEx(MIDDLE_FLEX_MOTOR, MONKEY_MIDDLE.front(), 3400, 50);
    st.RegWritePosEx(RING_LITTLE_FLEX_MOTOR, MONKEY_RING.front(), 3400, 50);
    st.RegWriteAction();
    delay(10);

    st.RegWritePosEx(THUMB_ROT_MOTOR, MONKEY_THUMB_ROT.front(), 3400, 50);
    st.RegWriteAction();
    delay(10);
  }

  void monkeyGrasp(const int factor) {
    byte ID[] = { INDEX_FLEX_MOTOR, MIDDLE_FLEX_MOTOR, RING_LITTLE_FLEX_MOTOR, THUMB_FLEX_MOTOR };
    u16 Speed[] = { 3400, 3400, 3400, 3400 };
    byte ACC[] = { 50, 50, 50, 50 };

    auto index_flex_joint_pos_value = map(factor, 0, 50, MONKEY_INDEX.front(), MONKEY_INDEX.back());
    index_flex_joint_pos_value = limit(index_flex_joint_pos_value, MONKEY_INDEX.front(), MONKEY_INDEX.back());

    auto middle_flex_joint_pos_value = map(factor, 0, 50, MONKEY_MIDDLE.front(), MONKEY_MIDDLE.back());
    middle_flex_joint_pos_value = limit(middle_flex_joint_pos_value, MONKEY_MIDDLE.front(), MONKEY_MIDDLE.back());

    auto ring_little_flex_joint_pos_value = map(factor, 0, 50, MONKEY_RING.front(), MONKEY_RING.back());
    ring_little_flex_joint_pos_value = limit(ring_little_flex_joint_pos_value, MONKEY_RING.front(), MONKEY_RING.back());

    auto thumb_flex_joint_pos_value = map(factor, 60, 100, MONKEY_THUMB.front(), MONKEY_THUMB.back());
    thumb_flex_joint_pos_value = limit(thumb_flex_joint_pos_value, MONKEY_THUMB.front(), MONKEY_THUMB.back());

    s16 Position[] = { index_flex_joint_pos_value, middle_flex_joint_pos_value, ring_little_flex_joint_pos_value, thumb_flex_joint_pos_value };

    auto t1 = micros();
    st.SyncWritePosEx(ID, 4, Position, Speed, ACC);  //servo(ID1/ID2) speed=3400, acc=50, move to position=3000.
    auto t2 = micros();
    auto dt = (t2 - t1) * 1.0 / 1000.0;
    //SerialBT.printf("SyncWritePosEx [ms]:  %f \n", dt);
  }


  void preparePinchGrasp() {
    st.RegWritePosEx(THUMB_FLEX_MOTOR, PINCH_THUMB.front(), 3400, 50);
    st.RegWriteAction();
    st.RegWritePosEx(INDEX_FLEX_MOTOR, PINCH_INDEX.front(), 3400, 50);
    st.RegWritePosEx(MIDDLE_FLEX_MOTOR, PINCH_MIDDLE.front(), 3400, 50);
    st.RegWritePosEx(RING_LITTLE_FLEX_MOTOR, PINCH_RING.front(), 3400, 50);

    delay(10);
    st.RegWritePosEx(THUMB_ROT_MOTOR, PINCH_THUMB_ROT.front(), 3400, 50);
    st.RegWriteAction();
    delay(10);
  }

  void pinchGrasp(const int factor) {

    byte ID[] = { INDEX_FLEX_MOTOR, MIDDLE_FLEX_MOTOR, RING_LITTLE_FLEX_MOTOR, THUMB_FLEX_MOTOR };
    u16 Speed[] = { 3400, 3400, 3400, 3400 };
    byte ACC[] = { 50, 50, 50, 50 };

    auto index_flex_joint_pos_value = map(factor, 30, 100, PINCH_INDEX.front(), PINCH_INDEX.back());
    index_flex_joint_pos_value = constrain(index_flex_joint_pos_value, PINCH_INDEX.front(), PINCH_INDEX.back());

    auto middle_flex_joint_pos_value = map(factor, 30, 100, PINCH_MIDDLE.front(), PINCH_MIDDLE.back());
    middle_flex_joint_pos_value = limit(middle_flex_joint_pos_value, PINCH_MIDDLE.front(), PINCH_MIDDLE.back());

    auto ring_little_flex_joint_pos_value = map(factor, 30, 100, PINCH_RING.front(), PINCH_RING.back());
    ring_little_flex_joint_pos_value = limit(ring_little_flex_joint_pos_value, PINCH_RING.front(), PINCH_RING.back());

    auto thumb_flex_joint_pos_value = map(factor, 0, 50, PINCH_THUMB.front(), PINCH_THUMB.back());
    thumb_flex_joint_pos_value = constrain(thumb_flex_joint_pos_value, PINCH_THUMB.front(), PINCH_THUMB.back());

    s16 Position[] = { index_flex_joint_pos_value, middle_flex_joint_pos_value, ring_little_flex_joint_pos_value, thumb_flex_joint_pos_value };

    st.SyncWritePosEx(ID, 4, Position, Speed, ACC);  //servo(ID1/ID2) speed=3400, acc=50, move to position=3000.
  }

  void moveIndex(const int factor) {
    byte ID[] = { INDEX_FLEX_MOTOR };
    u16 Speed[] = { 3400 };
    byte ACC[] = { 50 };
    auto index_flex_joint_pos_value = map(factor, 0, 100, FULLRANGE_INDEX.front(), FULLRANGE_INDEX.back());
    index_flex_joint_pos_value = constrain(index_flex_joint_pos_value, FULLRANGE_INDEX.front(), FULLRANGE_INDEX.back());
    s16 Position[] = { index_flex_joint_pos_value };
    st.SyncWritePosEx(ID, 1, Position, Speed, ACC);
  }

  void moveIndexSync(const int factor) {
    byte ID[] = { INDEX_FLEX_MOTOR };
    u16 Speed[] = { 3400 };
    byte ACC[] = { 50 };
    auto index_flex_joint_pos_value = map(factor, 0, 100, FULLRANGE_INDEX.front(), FULLRANGE_INDEX.back());
    index_flex_joint_pos_value = constrain(index_flex_joint_pos_value, FULLRANGE_INDEX.front(), FULLRANGE_INDEX.back());
    s16 Position[] = { index_flex_joint_pos_value };
    st.SyncWritePosEx(ID, 1, Position, Speed, ACC);

    int p = st.ReadPos(INDEX_FLEX_MOTOR);
    const int DYN_LOAD_POS_ERROR_THR = 50;
    while (abs(p - index_flex_joint_pos_value) > DYN_LOAD_POS_ERROR_THR) {
      p = st.ReadPos(INDEX_FLEX_MOTOR);
      auto load = st.ReadLoad(FingersController::INDEX_FLEX_MOTOR);
      SerialBT.printf("M p:%d  %d  L:%d\n", p, index_flex_joint_pos_value, load);
    }
  }

  int readPos(const int motorId) {
    return st.ReadPos(motorId);
  }

  void testCalib(const int motor_id, const int offset = 0) {
    st.WritePosEx(motor_id, 2048 + offset, 3400, 50);
  }

  void openIndex() {
    st.WritePosEx(INDEX_FLEX_MOTOR, FULLRANGE_INDEX.front(), 3400, 50);
  }

  void openMiddle() {
    st.WritePosEx(MIDDLE_FLEX_MOTOR, FULLRANGE_MIDDLE.front(), 3400, 50);
  }

  void openRingLittle() {
    st.WritePosEx(RING_LITTLE_FLEX_MOTOR, FULLRANGE_RING.front(), 3400, 50);
  }

  void openThumb() {
    st.WritePosEx(THUMB_FLEX_MOTOR, FULLRANGE_THUMB.front(), 3400, 50);
  }

  void openThumbRot() {
    st.WritePosEx(THUMB_ROT_MOTOR, FULLRANGE_THUMB_ROT.front(), 3400, 50);
  }

  void closeIndex() {
    st.WritePosEx(INDEX_FLEX_MOTOR, FULLRANGE_INDEX.back(), 3400, 50);
  }

  void closeMiddle() {
    st.WritePosEx(MIDDLE_FLEX_MOTOR, FULLRANGE_MIDDLE.back(), 3400, 50);
  }

  void closeRingLittle() {
    st.WritePosEx(RING_LITTLE_FLEX_MOTOR, FULLRANGE_RING.back(), 3400, 50);
  }

  void closeThumb() {
    st.WritePosEx(THUMB_FLEX_MOTOR, FULLRANGE_THUMB.back(), 3400, 50);
  }

  void closeThumbRot() {
    st.WritePosEx(THUMB_ROT_MOTOR, FULLRANGE_THUMB_ROT.back(), 3400, 50);
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

  void prepareGrasp(const GraspType grasp_type) {
    switch (grasp_type) {
      case GraspType::POWER:
        preparePowerGrasp();
        // preparePowerToolGrasp();
        break;
      case GraspType::POWERSMALL:
        preparePowerSmallGrasp();
        break;
      case GraspType::MONKEY:
        prepareMonkeyGrasp();
        break;
      case GraspType::PINCH:
        preparePinchGrasp();
        break;
      case GraspType::RELAX:
        prepareRelax();
        break;
    }
  }

  void grasp(const FingersController::GraspType grasp_type, const int factor, const bool sync = false, const int load_limit = 1000) {
    switch (grasp_type) {
      case FingersController::GraspType::POWER:
        powerGrasp(factor);
        // powerToolGrasp(factor);
        break;
      case FingersController::GraspType::POWERSMALL:
        //powerSmallGrasp(factor, sync, load_limit);
        powerSmallGraspSync(factor);
        break;
      case FingersController::GraspType::MONKEY:
        monkeyGrasp(factor);
        break;
      case FingersController::GraspType::PINCH:
        pinchGrasp(factor);
        break;
      case FingersController::GraspType::RELAX:
        relaxGrasp(factor);
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

  void feedback() {
    int Pos;
    int Speed;
    int Load;
    int Voltage;
    int Temper;
    int Move;
    int Current;
    if (st.FeedBack(1) != -1) {
      Pos = st.ReadPos(5);
      SerialBT.print("POS: ");
      SerialBT.println(Pos);
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

public:
  // Motors
  static const int INDEX_FLEX_MOTOR = 2;
  static const int MIDDLE_FLEX_MOTOR = 1;
  static const int RING_LITTLE_FLEX_MOTOR = 3;
  static const int THUMB_ROT_MOTOR = 5;
  static const int THUMB_FLEX_MOTOR = 4;

private:
  // Positions
  // Convention: from open state to close state
  // TODO Finger class
  const std::vector<int> FULLRANGE_INDEX = { 600, 3500 };
  const std::vector<int> FULLRANGE_MIDDLE = { 3400, 100 };
  const std::vector<int> FULLRANGE_RING = { 4000, 100 };
  const std::vector<int> FULLRANGE_THUMB = { 700, 3000 };
  const std::vector<int> FULLRANGE_THUMB_ROT = { 1500, 2400 };

  const std::vector<int> POWER_INDEX = { 600, 3300 };       // PLA 3800, TPU 2100
  const std::vector<int> POWER_MIDDLE = { 3000, 100 };      // PLA 100, TPU 1950
  const std::vector<int> POWER_RING = { 3800, 400 };        // PLA 100, TPU 1950
  const std::vector<int> POWER_THUMB = { 700, 2600 };       // PLA 2600, TPU 2600
  const std::vector<int> POWER_THUMB_ROT = { 2300, 2300 };  // PLA 2300, TPU 2300

  const std::vector<int> POWERSMALL_INDEX = { 1250, 4000 };      // PLA 3800
  const std::vector<int> POWERSMALL_MIDDLE = { 2500, 100 };      // PLA 100
  const std::vector<int> POWERSMALL_RING = { 3100, 100 };        // PLA 100
  const std::vector<int> POWERSMALL_THUMB = { 1100, 2600 };      // PLA 2600
  const std::vector<int> POWERSMALL_THUMB_ROT = { 1900, 1900 };  // PLA 1900

  const std::vector<int> POWERTOOL_INDEX = { 600, 3900 };       // PLA 3800
  const std::vector<int> POWERTOOL_MIDDLE = { 3000, 100 };      // PLA 100
  const std::vector<int> POWERTOOL_RING = { 3800, 100 };        // PLA 100
  const std::vector<int> POWERTOOL_THUMB = { 700, 3000 };       // PLA 2600
  const std::vector<int> POWERTOOL_THUMB_ROT = { 2300, 2300 };  // PLA 1900

  const std::vector<int> MONKEY_INDEX = { 600, 3700 };       // PLA 3800
  const std::vector<int> MONKEY_MIDDLE = { 3000, 100 };      // PLA 100
  const std::vector<int> MONKEY_RING = { 3800, 100 };        // PLA 100
  const std::vector<int> MONKEY_THUMB = { 700, 3500 };       // PLA 3500
  const std::vector<int> MONKEY_THUMB_ROT = { 1500, 1500 };  // PLA 1500

  const std::vector<int> PINCH_INDEX = { 1250, 2500 };      // PLA 3800
  const std::vector<int> PINCH_MIDDLE = { 2500, 1000 };     // PLA 3200
  const std::vector<int> PINCH_RING = { 100, 100 };         // PLA 2600
  const std::vector<int> PINCH_THUMB = { 700, 1800 };       // PLA 2050
  const std::vector<int> PINCH_THUMB_ROT = { 2250, 2250 };  // PLA 2100

  const std::vector<int> RELAX_INDEX = { 1340, 1340 };
  const std::vector<int> RELAX_MIDDLE = { 2200, 2200 };
  const std::vector<int> RELAX_RING = { 3000, 3000 };
  const std::vector<int> RELAX_THUMB = { 2300, 2300 };
  const std::vector<int> RELAX_THUMB_ROT = { 2100, 2100 };

  const bool DEBUG_LOG = true;

private:  // load control
  const int DYN_LOAD_POS_ERROR_THR = 50;
  const int MIN_CLOSURE_FOR_LOAD_CTRL = 30;

private:
  SMS_STS st;
  BluetoothSerial SerialBT;
};
