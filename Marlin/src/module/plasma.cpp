/**
 * Plasma CNC
 * Copyright 2021 Claes Jegerås
 * 
 * Plasma control
 * Torch Height Controller
 * Gcode support (M21XX)
 * 
 * 
 * 
 */

/**
 * plasma.cpp
 *
 * Manage digital CPC signal from plasma cutter: START and TRANSFER
 *
 * 
 * Copyright 2021 Claes Jegerås
 */
#include "../MarlinCore.h"

#if ANY(MPCNC_PLASMA, COREXY_PLASMA)

#include "plasma.h"
#include "stepper.h"
#include "endstops.h"
#include "planner.h"

Plasma plasmaManager;

/**
 * Class and Instance Methods
 */

Plasma::Plasma() { }

#if PLASMA_THC
VoltageInfo Plasma::voltage_minus = VoltageInfo();
VoltageInfo Plasma::voltage_plus = VoltageInfo();

uint16_t Plasma::voltageDivider = uint16_t();
uint16_t Plasma::wantedThcVoltage = uint16_t();

bool Plasma::runThc = bool();
#endif

/**
 * Get average (avr) voltages
 */
#if PLASMA_THC
void Plasma::SetCurrentVoltageAverage() {
  static uint16_t meanVoltage = 0;
  static uint16_t loopCount = 0;

  // Help to determine OFFSET and SLOPE
  
  meanVoltage += (voltage_plus.acc - voltage_minus.acc);
  loopCount++;
  if (loopCount == 50) {
    meanVoltage /= loopCount;
    loopCount = 0;
    SERIAL_ECHOLNPAIR("mean VD: ", (meanVoltage - OFFSET)/SLOPE);
  }
  

  voltageDivider = (voltage_plus.acc - voltage_minus.acc)/OVERSAMPLENR - OFFSET;
  if (voltageDivider > 1024) {
	  voltageDivider = 0;
  }
  voltage_plus.acc = 0;
  voltage_minus.acc = 0;
  // Resolution is 0.200 mV, so use the voltageDivider value as is.
}

uint16_t Plasma::getActualThcVoltage() {

	return voltageDivider;
}

uint16_t Plasma::getWantedThcVoltage() {
  return wantedThcVoltage;
}

void Plasma::setWantedThcVoltage(uint16_t voltage_) {
  wantedThcVoltage = voltage_;
}
void Plasma::enableThc(void) {
  //stepper.leave_control_on(Z_AXIS);
  runThc = true;

  SERIAL_ECHOLN("enableTHC");
};
void Plasma::disableThc(void) {
  //stepper.take_control_on(Z_AXIS);
  runThc = false;
  SERIAL_ECHOLN("disableTHC");
};

void Plasma::updateThc(void) {

  // Get difference between set and actual voltage
  // Based on the difference, step Z a of number steps
  // Check boundaries form initial height usually 1.5 mm,
  // to let say 10 mm. If out of bounds, disableThc and also let PLASMA START go.

};

void HAL_VOLTAGE_TIMER_ISR() {
  HAL_timer_isr_prologue(VOLTAGE_TIMER_NUM);

  Plasma::isr();

  HAL_timer_isr_epilogue(VOLTAGE_TIMER_NUM);
}


void Plasma::isr() {

  static int8_t sampleCount = 0;
  static MeasureVoltageState adcSensorState = MeasureVoltagePlus;

  switch (adcSensorState) {

    case MeasureVoltagePlus:
      Plasma::voltage_plus.acc += HAL_READ_ADC();
      HAL_START_ADC(VOLTAGE_DIVIDER_MINUS_PIN);
      adcSensorState = MeasureVoltageMinus;
      break;

    case MeasureVoltageMinus:
      Plasma::voltage_minus.acc += HAL_READ_ADC();
      HAL_START_ADC(VOLTAGE_DIVIDER_PLUS_PIN);

      sampleCount++;
      if (sampleCount >= OVERSAMPLENR) {
        sampleCount = 0;
        TOGGLE(PLASMA_VD_UPDATES_PIN); // To see the sample time on an oscilloscope.
        Plasma::SetCurrentVoltageAverage(); // With OVERSAMPLENR = 1, we get 2 ms
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
#endif
#endif
