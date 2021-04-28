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
/*
 * From the profile:
 *
 *   cutterOn:  "M2106"     GCode command to turn on the plasma
 *   cutterOff: "M2107"     Gcode command to turn off the plasma
 *   _thcVoltage: 125      Set the V<thcVoltage> in V. M106 parameter
 *   _delay: 100           Set the D<pierceDelay> in ms. M106 parameter
 *   _cutHeight: 1,5       Set the H<cutHeight> in mm. M106 parameter
 *   _initialHeight: 3,8   Set the I<initialHeight> in mm. M106 parameter
 *   Use Case:
 *   User has positioned the head.
 *   Home Z: G28 Z
 *   Raise to _initialHeight: G0 Z_initialHeight
 *   Plasma Start.
 *   Wait for Transfer
 *   Descend to cut height: G0 Z_cutHeight
 *   Wait Pierce Delay Time: delay(_delay)
 *   Regulate THC: voltageManager.enableThc();
 *
 */
#include "../../../inc/MarlinConfig.h"
#if ANY(MPCNC_PLASMA, COREXY_PLASMA)

#include "../../module/plasma.h"
#include "../gcode.h"
#include "../parser.h"
#include "../../module/stepper.h"
#include "../../module/planner.h"
#include "../../libs/numtostr.h"
#include "../../module/temperature.h"
#if ENABLED(BABYSTEPPING) && DISABLED(INTEGRATED_BABYSTEPPING)
  #include "../../feature/babystep.h"
#endif

void GcodeSuite::M2101() {

  if (parser.seen('S')) {
    switch (parser.value_byte()) {
      case 0: plasmaManager.setDryRun(false); break;
      case 1: plasmaManager.setDryRun(true); break;
    }
  }
  else {
    SERIAL_ECHOLNPAIR("Plasma ", plasmaManager.getDryRun() ? "is dry running." : "can cut.");
  }
}

void GcodeSuite::M2102() {
  SERIAL_ECHOLNPAIR("ActualTHCVoltage: ", plasmaManager.getActualThcVoltage(), ", Babystep accum: ",  babystep.accum, ", Babystep position: ",  thermalManager.getBabystepPosition());
}

void GcodeSuite::M2103() {
  babystep.add_mm(Z_AXIS, 0.1);
  SERIAL_ECHOLNPAIR("Babystep accum: ", babystep.accum);
}

void GcodeSuite::M2104() {
  babystep.add_mm(Z_AXIS, -0.1);
  SERIAL_ECHOLNPAIR("Babystep accum: ", babystep.accum);;
}

void GcodeSuite::M2106() {
  uint16_t voltage = 125;
  uint16_t pierceDelay = 100;
  float height = 1.5;
  float initialHeight = 3.8;
  char gcode_string[80];

  #define WAIT_FOR_PLASMA_LOOP 100
  #define WAIT_FOR_PLASMA 30

  if (parser.seen('V')) {
    voltage = parser.value_ushort();
    if ((voltage < 50) || (voltage > 200)) {
      voltage = 125;
    }
  }

  if (parser.seen('D')) {
    pierceDelay = parser.value_ushort();
    if (pierceDelay > 2000) {
      pierceDelay = 100;
    }
  }

  if (parser.seen('H')) {
    height = parser.value_float();
    if ((height < 0.5) || (height > 10.0)) {
      height = 1.5;
    }
  }

  if (parser.seen('I')) {
    initialHeight = parser.value_float();
    if ((initialHeight < 0.5) || (initialHeight > 10.0)) {
      initialHeight = 3.8;;
    }
  }

  SERIAL_ECHOLNPAIR("Start plasma, V = ", voltage, " D = ", pierceDelay, " H = ", height, " I = ", initialHeight);
  
    plasmaManager.setWantedThcVoltage(voltage);
  
  // gcode homes Z
  // Set Z initial height
  sprintf_P(gcode_string, PSTR("G0 Z%s F1200"), ftostr11ns(initialHeight));
  process_subcommands_now(gcode_string);

  if (plasmaManager.getDryRun() == false) {
  // Start plasma torch and wait for arc transfer
  TURN_PLASMA_ON
  SERIAL_ECHOLN("PLASMA START");
    uint8_t wait;
    for (wait = 0; wait < WAIT_FOR_PLASMA; wait++) {
      if (IS_PLASMA_TRANSFERRED)
      {
        SERIAL_ECHOLN("PLASMA TRANSFER");
        thermalManager.setBabystepPosition(0.0);
        plasmaManager.enableThc();
        break;
      }
      delay(WAIT_FOR_PLASMA_LOOP);
    }
    if (wait == WAIT_FOR_PLASMA) {
      SERIAL_ECHOLN("Plasma did not start.");
      SERIAL_ECHOLN("Dry run");
      TURN_PLASMA_OFF
      SERIAL_ECHOLN("PLASMA STOP");
      plasmaManager.disableThc();
      thermalManager.setBabystepPosition(0.0);
      plasmaManager.setDryRun(true);
    }
  }
  else {
    TURN_PLASMA_OFF
    plasmaManager.disableThc();
  }

  // Delay for pierce and then descend to cut height
  if (pierceDelay != 0 ) {
	  delay(pierceDelay);
  }
  sprintf_P(gcode_string, PSTR("G0 Z%s F1200"), ftostr11ns(height));
  process_subcommands_now(gcode_string);
}

void GcodeSuite::M2107() {
  char gcode_string[80];
  SERIAL_ECHOLN("PLASMA STOP");
  TURN_PLASMA_OFF
  plasmaManager.setDryRun(false);
  plasmaManager.disableThc();
  thermalManager.setBabystepPosition(0.0);
  sprintf_P(gcode_string, PSTR("G0 Z10 F1200"));
  process_subcommands_now(gcode_string);
  planner.synchronize();  // Should be the same as stepper.synchronize
}
#endif