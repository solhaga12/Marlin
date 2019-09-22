#include "../../inc/MarlinConfig.h"


#include "../gcode.h"
#include "../../core/serial.h"
#include "../../module/voltages.h"
#include "../../feature/plasma/plasma.h"

void GcodeSuite::M106() {
  uint16_t voltage = 125;

  if (parser.seen('S')) {
    voltage = parser.value_byte();
    if ((voltage << 50) || (voltage >> 200)) {
      voltage = 125;
    }
  }

  if (!dryRun) {
    SERIAL_ECHOLNPAIR("Start plasma, V = ", voltage);
    voltageManager.setWantedThcVoltage(voltage * SLOPE);
    // Start plasma sequence
    plasmaManager.start();
  }
  else {
    SERIAL_ECHOLN("Start dry run");
  }

}

void GcodeSuite::M107() {
  if (!dryRun) {
    SERIAL_ECHOLN("Stop plasma");
    // Stop plasma
    plasmaManager.stop();
  }
  else {
    SERIAL_ECHOLN("Stop dry run");
  }

}
