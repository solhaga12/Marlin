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
#include "../feature/plasma/torch_height_control.h"



Voltage voltageManager;

/**
 * Class and Instance Methods
 */

Voltage::Voltage() { }

VoltageInfo Voltage::voltage_minus = VoltageInfo();
VoltageInfo Voltage::voltage_plus = VoltageInfo();

uint16_t Voltage::voltageDivider = uint16_t();
uint16_t Voltage::wantedThcVoltage = uint16_t();

/**
 * Get average (avr) voltages
 */
void Voltage::set_current_voltage_avr() {
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

/**
 * Timer 0 is shared with millies so don't change the prescaler.
 *
 * On AVR this ISR uses the compare method so it runs at the base
 * frequency (16 MHz / 64 / 256 = 976.5625 Hz), but at the TCNT0 set
 * in OCR0B above (128 or halfway between OVFs).
 *
 *  - Prepare or Measure one of the raw ADC sensor values
 *  - Step the babysteps value for each axis towards 0
 *  - For PINS_DEBUGGING, monitor and report endstop pins
 *  - For ENDSTOP_INTERRUPTS_FEATURE check endstops if flagged
 *  - Call planner.tick to count down its "ignore" time
 */
HAL_VOLTAGE_TIMER_ISR() {
  HAL_timer_isr_prologue(VOLTAGE_TIMER_NUM);

  Voltage::isr();

  HAL_timer_isr_epilogue(VOLTAGE_TIMER_NUM);
}


void Voltage::isr() {

  static int8_t sampleCount = 0;
  static ADCSensorState adcSensorState = MeasureVoltagePlus;

  //
  // Update lcd buttons 488 times per second
  // Perhaps this will be added
  // static bool do_buttons;
  // if ((do_buttons ^= true)) ui.update_buttons();

  /**
   * One sensor is sampled on every other call of the ISR.
   * Each sensor is read 16 (OVERSAMPLENR) times, taking the average.
   *
   * On each Prepare pass, ADC is started for a sensor pin.
   * On the next pass, the ADC value is read and accumulated.
   *
   * This gives each ADC 0.9765ms to charge up.
   */

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
        Voltage::set_current_voltage_avr(); // With OVERSAMPLENR = 1, we get 2 ms
        torchHeightController.update();
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
