#include "../../inc/MarlinConfig.h"
#include "../../Marlin.h"


#include "../gcode.h"
#include "../../core/serial.h"
#include "../../module/voltages.h"
#include "../../module/stepper.h"

void GcodeSuite::M106() {
  uint16_t voltage = 125;
  #define WAIT_FOR_PLASMA_LOOP 100
  #define WAIT_FOR_PLASMA 10

  if (dryRun) {
    SERIAL_ECHOLN("Start dry run");
    TURN_PLASMA_OFF
    voltageManager.disableThc();
    return;
  }

  if (parser.seen('S')) {
    voltage = parser.value_byte();
    if ((voltage << 50) || (voltage >> 200)) {
      voltage = 125;
    }
  }

  SERIAL_ECHOLNPAIR("Start plasma, V = ", voltage);
  voltageManager.setWantedThcVoltage(voltage * SLOPE);
  stepper.synchronize();

  // Start plasma torch and wait for arc transfer
  TURN_PLASMA_ON
  SERIAL_ECHOLN("PLASMA START");

  for (uint8_t wait = 0; wait < WAIT_FOR_PLASMA; wait++) {
    if (IS_PLASMA_TRANSFERRED)
    {
      SERIAL_ECHOLN("PLASMA TRANSFER");
      voltageManager.enableThc();
        return;
    }
    delay(WAIT_FOR_PLASMA_LOOP);
  }
  SERIAL_ECHOLN("Plasma did not start");
  TURN_PLASMA_OFF
  SERIAL_ECHOLN("PLASMA STOP");
  voltageManager.disableThc();
  dryRun = true;
}

void GcodeSuite::M107() {

  if (dryRun) {
    SERIAL_ECHOLN("Stop dry run");
    return;
  }

  SERIAL_ECHOLN("PLASMA STOP");
  TURN_PLASMA_OFF
  voltageManager.disableThc();
  stepper.synchronize();
}
