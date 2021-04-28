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
#pragma once
/**
 * plasma.h
 *
 * Manage digital CPC signal from plasma cutter: START and TRANSFER
 *
 * 
 * Copyright 2021 Claes Jegerås
 */

#include "../MarlinCore.h"
#if ANY(MPCNC_PLASMA, COREXY_PLASMA)

#define TURN_PLASMA_ON WRITE(PLASMA_START_PIN, HIGH);
#define TURN_PLASMA_OFF WRITE(PLASMA_START_PIN, LOW);
#define IS_PLASMA_TRANSFERRED READ(PLASMA_TRANSFER_PIN) ? LOW : HIGH

class Plasma {

  public:
    static uint16_t getWantedThcVoltage(void);
    static void setWantedThcVoltage(uint16_t voltage);
    static float_t getActualThcVoltage(void);
    static void setActualThcVoltage(float_t voltage);
    static void enableThc(void);
    static void disableThc(void);
    static bool isThcEnabled(void);
    static void setDryRun(bool dryRun);
    static bool getDryRun(void);

  private:
    static uint16_t wantedThcVoltage;
    static float_t actualThcVoltage;

    static bool dryRun;
    static bool runThc;

  public:
    /**
     * Instance Methods
     */

    Plasma();

    /**
     * Initialize the plasma manager
     * The manager is implemented by periodic calls to manage_voltage()
     */
    void init() {

      SET_OUTPUT(PLASMA_START_PIN);
      TURN_PLASMA_OFF;
      SET_OUTPUT(PLASMA_VD_UPDATES_PIN);
      SET_INPUT_PULLUP(PLASMA_TRANSFER_PIN);
      SET_OUTPUT(PLASMA_DEBUG_1_PIN);
      SET_OUTPUT(PLASMA_DEBUG_1_PIN);

      dryRun = false;
      runThc = false;
    }
};

extern Plasma plasmaManager;
#endif