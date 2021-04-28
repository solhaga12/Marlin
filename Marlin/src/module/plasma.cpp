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

uint16_t Plasma::wantedThcVoltage = uint16_t();
float_t Plasma::actualThcVoltage = float_t();

bool Plasma::dryRun = bool();
bool Plasma::runThc = bool();


void Plasma::setDryRun(bool _dryRun) {

	dryRun = _dryRun;
}

bool Plasma::getDryRun() {
  return dryRun;
}

/**
 * Get average (avr) voltages
 */
uint16_t Plasma::getWantedThcVoltage() {
  return wantedThcVoltage;
}

void Plasma::setWantedThcVoltage(uint16_t voltage_) {
  wantedThcVoltage = voltage_;
}
float_t Plasma::getActualThcVoltage() {
  return actualThcVoltage;
}

void Plasma::setActualThcVoltage(float_t voltage_) {
  actualThcVoltage = voltage_;
}
void Plasma::enableThc(void) {
  runThc = true;
  SERIAL_ECHOLN("enableTHC");
};
void Plasma::disableThc(void) {
  runThc = false;
  SERIAL_ECHOLN("disableTHC");
};
bool Plasma::isThcEnabled(void) {
  return runThc;
};
#endif
