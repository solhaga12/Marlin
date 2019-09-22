#include "../../inc/MarlinConfig.h"


#include "../gcode.h"
#include "../../core/serial.h"
#include "../../module/voltages.h"
#include "../../feature/plasma/plasma.h"
#include "../../stepper.h"

void GcodeSuite::M106() {
  uint16_t voltage = 125;

  if (dryRun) {
    SERIAL_ECHOLN("Start dry run");
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
  while (true) {
    refresh_cmd_timeout();
    millis_t transfer_timeout = PLASMA_TRANSFER_TIMEOUT_MS;
    transfer_timeout += previous_cmd_ms;

    if(!plasmaManager.start()) return;

    KEEPALIVE_STATE(PAUSED_FOR_INPUT);
    while (PENDING(millis(), transfer_timeout)) {
      PlasmaState plasma_state = plasmaManager.get_state();
      if(IS_WAITING_FILE)
      {
        return;
      }
      if(plasma_state == Established)
      {
        lcd_setstatus("Running...");
        SERIAL_ECHOPAIR("Plasma started");
        return;
      }
      if(IS_SUSPENDED)
        break;
      idle(true);
    }
    plasmaManager.stop();

    if(stateManager.suspend())
      lcd_setstatus("Paused, plasma error.");

    while (IS_SUSPENDED) idle();
    if(IS_WAITING_FILE)
      return;

    lcd_setstatus("Retry ignition...");

  }
}

void GcodeSuite::M107() {

  if (dryRun) {
    SERIAL_ECHOLN("Stop dry run");
  }

    SERIAL_ECHOLN("Stop plasma");
    // Stop plasma
    plasmaManager.stop();

}
