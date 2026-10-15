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

#include "ESP32_fft.h"  // include the library

// EMG
const int EMG_CLOSE_PIN = 34;  //33;  //34;
const int EMG_OPEN_PIN = 35;   //32;   //35;
int EMG_CLOSE_THR = 1000;      //esp32 100000 ; arduino 8500;
int EMG_OPEN_THR = 1000;       //esp32 100000; arduino 15000;
const int emgPins[2] = { EMG_OPEN_PIN, EMG_CLOSE_PIN };

class FFTManager {
public:
  FFTManager() {
    esp_fft_ = new ESP_fft(FFT_NN, SAMPLEFREQ, FFT_REAL, FFT_FORWARD, fft_input_, fft_output_);
  }

  float process(int analog_input_pin) {

    // read signal
    for (int k = 0; k < FFT_NN; k++) {
      fft_input_[k] = (float)analogRead(analog_input_pin);
    }

    // perform FFT
    esp_fft_->removeDC();
    esp_fft_->hammingWindow();
    esp_fft_->execute();
    esp_fft_->complexToMagnitude();
    
    return esp_fft_->getMagAtBin(bin200hz);
  }

private:
  ESP_fft* esp_fft_;

  static const int FFT_NN = 512;       // Must be a power of 2
  static const int SAMPLEFREQ = 9901;  //
  static const int bin200hz = 11;
  static const int bin400hz = 22;

  float fft_input_[FFT_NN];
  float fft_output_[FFT_NN];

  BluetoothSerial SerialBT;
};
