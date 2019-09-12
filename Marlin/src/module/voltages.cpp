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
#include "voltages.h"
#include "../HAL/HAL_AVR/HAL.h"
#include "../HAL/shared/Delay.h"
#include "../libs/numtostr.h"
#include "../core/serial.h"


Voltage voltageManager;

/**
 * Class and Instance Methods
 */

Voltage::Voltage() { }

voltage_info_t voltage_plus = voltage_info_t();
voltage_info_t voltage_minus = voltage_info_t();


/**
 * Get avr voltages
 */
void Voltage::set_current_voltage_avr() {

  voltageDivider = (voltage_plus.acc - voltage_minus.acc)/OVERSAMPLENR;
  if (voltageDivider < 0) {
	  voltageDivider = 0;
  }
  voltage_plus.acc = 0;
  voltage_minus.acc = 0;

  SERIAL_ECHOLN(voltageDivider);
}

uint16_t Voltage::getVoltage() {

	return voltageDivider;
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

  static int8_t voltage_count = -1;
  static ADCSensorState adc_sensor_state = StartupDelay;

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

  // SERIAL_ECHOLNPAIR("ISR ",adc_sensor_state);

  switch (adc_sensor_state) {

  	case StartupDelay:
  		// Possible delays
        adc_sensor_state = PrepareVoltagePlus;
      break;

    case PrepareVoltagePlus:
      HAL_START_ADC(PLASMA_VOLTAGE_DIVIDER_PLUS_PIN);
      adc_sensor_state = MeasureVoltagePlus;
      break;

    case MeasureVoltagePlus:
      Voltage::voltage_plus.acc += HAL_READ_ADC();
      adc_sensor_state = PrepareVoltageMinus;
      break;

    case PrepareVoltageMinus:
      HAL_START_ADC(PLASMA_VOLTAGE_DIVIDER_MINUS_PIN);
      adc_sensor_state = MeasureVoltageMinus;
      break;

    case MeasureVoltageMinus:
      Voltage::voltage_minus.acc += HAL_READ_ADC();
      adc_sensor_state = AverageVoltage;
      break;

    case AverageVoltage:                                   // Start of sampling loops. Do updates/checks.
      if (++voltage_count >= OVERSAMPLENR) {                 // 10 * 16 * 1/(16000000/64/256)  = 164ms.
        voltage_count = 0;
        Voltage::set_current_voltage_avr();
      }
      adc_sensor_state = StartupDelay;
      break;

  } // switch(adc_sensor_state)

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
