#include "../../inc/MarlinConfig.h"
#include "../../Marlin.h"


#include "../gcode.h"
#include "../queue.h"
#include "../../core/serial.h"
#include "../../module/voltages.h"
#include "../../module/stepper.h"
#include "../../libs/numtostr.h"

/*
 * From the profile:
 *
 *   cutterOn:  "M106"     GCode command to turn on the plasma
 *   cutterOff: "M107"     Gcode command to turn off the plasma
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

void GcodeSuite::M106() {
  uint16_t voltage = 125;
  uint16_t pierceDelay = 100;
  float height = 1.5;
  float initialHeight = 3.8;
  char gcode_string[80];

  #define WAIT_FOR_PLASMA_LOOP 100
  #define WAIT_FOR_PLASMA 30

  if (parser.seen('V')) {
    voltage = parser.value_byte();
    if ((voltage < 50) || (voltage > 200)) {
      voltage = 125;
    }
  }

  if (parser.seen('D')) {
    pierceDelay = parser.value_byte();
    if (pierceDelay > 2000) {
      pierceDelay = 100;
    }
  }

  if (parser.seen('H')) {
    height = parser.value_byte();
    if ((height < 0.5) || (height > 10.0)) {
      height = 1.5;
    }
  }

  if (parser.seen('I')) {
    initialHeight = parser.value_byte();
    if ((initialHeight < 0.5) || (initialHeight > 10.0)) {
      initialHeight = 3.8;;
    }
  }

  SERIAL_ECHOLNPAIR("Start plasma, V = ", voltage, " D = ", pierceDelay, " H = ", height, " I = ", initialHeight);

  voltageManager.setWantedThcVoltage(voltage * SLOPE);

  // Home Z and set initial height
  // sprintf_P(gcode_string, PSTR("G28 Z"));
  // process_subcommands_now(gcode_string);
  sprintf_P(gcode_string, PSTR("G0 Z%s F1800"), ftostr11ns(initialHeight));
  process_subcommands_now(gcode_string);

  if (!dryRun) {
  // Start plasma torch and wait for arc transfer
  TURN_PLASMA_ON
  SERIAL_ECHOLN("PLASMA START");

    for (uint8_t wait = 0; wait < WAIT_FOR_PLASMA; wait++) {
      if (IS_PLASMA_TRANSFERRED)
      {
        SERIAL_ECHOLN("PLASMA TRANSFER");
        voltageManager.enableThc();
        break;
      }
      delay(WAIT_FOR_PLASMA_LOOP);
    }
    if (!IS_PLASMA_TRANSFERRED) {
      SERIAL_ECHOLN("Plasma did not start");
      TURN_PLASMA_OFF
      SERIAL_ECHOLN("PLASMA STOP");
      voltageManager.disableThc();
      dryRun = true;
    }
  }
  else {
    SERIAL_ECHOLN("Start dry run");
    TURN_PLASMA_OFF
    voltageManager.disableThc();
  }

  // Delay for pierce and then descend to cut height
  if (pierceDelay != 0 ) {
	  delay(pierceDelay);
  }
  sprintf_P(gcode_string, PSTR("G0 Z%s F1200"), ftostr11ns(height));
  process_subcommands_now(gcode_string);
}

void GcodeSuite::M107() {

  if (dryRun) {
    SERIAL_ECHOLN("Stop dry run");
    dryRun = false;
    return;
  }

  SERIAL_ECHOLN("PLASMA STOP");
  TURN_PLASMA_OFF
  voltageManager.disableThc();
  stepper.synchronize();
}
