/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * voltages.cpp - voltages control
 */

#include "voltages.h"
#include "endstops.h"

#include "../Marlin.h"
#include "../lcd/ultralcd.h"
#include "planner.h"
#include "../HAL/HAL_AVR/HAL.h"
#include "../HAL/shared/Delay.h"
#include "../libs/numtostr.h"
#include "../core/serial.h"
#include "stepper.h"
#include "../gcode/queue.h"

Voltage voltageManager;

/**
 * Class and Instance Methods
 */

Voltage::Voltage() { }

VoltageInfo Voltage::voltage_minus = VoltageInfo();
VoltageInfo Voltage::voltage_plus = VoltageInfo();

uint16_t Voltage::voltageDivider = uint16_t();
uint16_t Voltage::wantedThcVoltage = uint16_t();

bool Voltage::runThc = bool();

/**
 * Get average (avr) voltages
 */
void Voltage::SetCurrentVoltageAverage() {
  static uint16_t meanVoltage = 0;
  static uint16_t loopCount = 0;

  // Help to determine OFFSET and SLOPE
  /*
  meanVoltage += (voltage_plus.acc - voltage_minus.acc);
  loopCount++;
  if (loopCount == 50) {
    meanVoltage /= loopCount;
    loopCount = 0;
    SERIAL_ECHOLNPAIR("mean VD: ", (meanVoltage - OFFSET)/SLOPE);
  }
  */

  voltageDivider = (voltage_plus.acc - voltage_minus.acc)/OVERSAMPLENR - OFFSET;
  if (voltageDivider > 1024) {
	  voltageDivider = 0;
  }
  voltage_plus.acc = 0;
  voltage_minus.acc = 0;
  // Resolution is 0.200 mV, so use the voltageDivider value as is.
}

uint16_t Voltage::getActualThcVoltage() {

	return voltageDivider;
}

uint16_t Voltage::getWantedThcVoltage() {
  return wantedThcVoltage;
}

void Voltage::setWantedThcVoltage(uint16_t voltage_) {
  wantedThcVoltage = voltage_;
}
void Voltage::enableThc(void) {
  stepper.leave_control_on(Z_AXIS);
  runThc = true;

  SERIAL_ECHOLN("enableTHC");
};
void Voltage::disableThc(void) {
  stepper.take_control_on(Z_AXIS);
  runThc = false;
  SERIAL_ECHOLN("disableTHC");
};

void Voltage::updateThc(void) {

  // Get difference between set and actual voltage
  // Based on the difference, step Z a of number steps
  // Check boundaries form initial height usually 1.5 mm,
  // to let say 10 mm. If out of bounds, disableThc and also let PLASMA START go.

};

HAL_VOLTAGE_TIMER_ISR() {
  HAL_timer_isr_prologue(VOLTAGE_TIMER_NUM);

  Voltage::isr();

  HAL_timer_isr_epilogue(VOLTAGE_TIMER_NUM);
}


void Voltage::isr() {

  static int8_t sampleCount = 0;
  static ADCSensorState adcSensorState = MeasureVoltagePlus;

  switch (adcSensorState) {

    case MeasureVoltagePlus:
      Voltage::voltage_plus.acc += HAL_READ_ADC();
      HAL_START_ADC(PLASMA_VOLTAGE_DIVIDER_MINUS_PIN);
      adcSensorState = MeasureVoltageMinus;
      break;

    case MeasureVoltageMinus:
      Voltage::voltage_minus.acc += HAL_READ_ADC();
      HAL_START_ADC(PLASMA_VOLTAGE_DIVIDER_PLUS_PIN);

      sampleCount++;
      if (sampleCount >= OVERSAMPLENR) {
        sampleCount = 0;
        TOGGLE(PLASMA_VD_UPDATES_PIN); // To see the sample time on an oscilloscope.
        Voltage::SetCurrentVoltageAverage(); // With OVERSAMPLENR = 1, we get 2 ms
        if (runThc) updateThc();
      }
      adcSensorState = MeasureVoltagePlus;
      break;

  } // switch(adcSensorState)

  //
  // Additional ~1kHz Tasks
  //

  #if ENABLED(BABYSTEPPING)
    babystep.task();
  #endif

  // Poll endstops state, if required
  endstops.poll();

  // Periodically call the planner timer
  planner.tick();


}
