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
 * Adopted to read voltage divider from plasma cutter.
 *
 */
#pragma once

/**
 * voltages.h - voltage controller
 */

#include "../inc/MarlinConfig.h"

/**
 * States for ADC reading in the ISR
 */
enum ADCSensorState : char {
  MeasureVoltagePlus,
  MeasureVoltageMinus
};

// Minimum number of Voltage::ISR loops between sensor readings.
// Multiplied by 16 (OVERSAMPLENR) to obtain the total time to
// get all oversampled sensor readings
// #define MIN_ADC_ISR_LOOPS 10

// #define ACTUAL_ADC_SAMPLES _MAX(int(MIN_ADC_ISR_LOOPS), int(SensorsReady))

// #define G26_CLICK_CAN_CANCEL (HAS_LCD_MENU && ENABLED(G26_MESH_VALIDATION))

// A voltage sensor
typedef struct VoltageInfo {
  uint16_t acc;
  uint16_t avr;
} voltage_info_t;

#define OVERSAMPLENR 8

class Voltage {

  public:

    static float getVoltage();

  private:

    static voltage_info_t voltage_plus;
    static voltage_info_t voltage_minus;
    static uint16_t voltageDivider;
    static float voltageReal;

  public:
    /**
     * Instance Methods
     */

    Voltage();

    /**
     * Initialize the voltage manager
     * The manager is implemented by periodic calls to manage_voltage()
     */
    void Voltage::init() {

      HAL_adc_init();

      analogReference(INTERNAL2V56);
      analogRead(0);

      HAL_ANALOG_SELECT(PLASMA_VOLTAGE_DIVIDER_PLUS_PIN);
      HAL_ANALOG_SELECT(PLASMA_VOLTAGE_DIVIDER_MINUS_PIN);

      HAL_timer_start(VOLTAGE_TIMER_NUM, VOLTAGE_TIMER_FREQUENCY);
      SERIAL_ECHOLN(VOLTAGE_TIMER_FREQUENCY);
      ENABLE_VOLTAGE_INTERRUPT();
    }

    /**
     * Static (class) methods
     */

      /**
     * Called from the Voltage ISR
     */
    static void isr();

  private:
    static void set_current_voltage_avr();
};
