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
/**
 * States for ADC reading in the ISR
 */
#if PLASMA_THC
enum MeasureVoltageState : char {
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

#define OVERSAMPLENR 1
#endif

#define TURN_PLASMA_ON WRITE(PLASMA_START_PIN, HIGH);
#define TURN_PLASMA_OFF WRITE(PLASMA_START_PIN, LOW);
#define IS_PLASMA_TRANSFERRED READ(PLASMA_TRANSFER_PIN) ? LOW : HIGH

class Plasma {

  public:
    #if PLASMA_THC
    static uint16_t getActualThcVoltage();
    static uint16_t getWantedThcVoltage();
    static void setWantedThcVoltage(uint16_t voltage);
    static void enableThc(void);
    static void disableThc(void);
    #endif
    static void setDryRun(bool dryRun);
    static bool getDryRun();


  private:
  #if PLASMA_THC
    static voltage_info_t voltage_plus;
    static voltage_info_t voltage_minus;
    static uint16_t voltageDivider;
    static uint16_t wantedThcVoltage;
    static bool runThc;

    #define OFFSET 23
    #define SLOPE 5.0
  #endif
    static bool dryRun;

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
      #if PLASMA_THC
      HAL_adc_init();

      analogReference(INTERNAL2V56);
      analogRead(0);

      HAL_ANALOG_SELECT(VOLTAGE_DIVIDER_PLUS_PIN);
      HAL_ANALOG_SELECT(VOLTAGE_DIVIDER_MINUS_PIN);

      HAL_timer_start(VOLTAGE_TIMER_NUM, VOLTAGE_TIMER_FREQUENCY);
      SERIAL_ECHOLN(VOLTAGE_TIMER_FREQUENCY);
      ENABLE_VOLTAGE_INTERRUPT();

      // Plasma
      HAL_ANALOG_SELECT(VOLTAGE_DIVIDER_PLUS_PIN);
      HAL_ANALOG_SELECT(VOLTAGE_DIVIDER_MINUS_PIN);
      #endif

      SET_OUTPUT(PLASMA_START_PIN);
      TURN_PLASMA_OFF;
      SET_OUTPUT(PLASMA_VD_UPDATES_PIN);
      SET_INPUT_PULLUP(PLASMA_TRANSFER_PIN);

      #if PLASMA_THC
      wantedThcVoltage = 100 * SLOPE;
      runThc = false;
      #endif
      dryRun = false;
    }

    /**
     * Static (class) methods
     */

      /**
     * Called from the Voltage ISR
     */
    #if PLASMA_THC
    static void isr();
    #endif

  private:
    #if PLASMA_THC
    static void SetCurrentVoltageAverage();
    static void updateThc();
    #endif
};

extern Plasma plasmaManager;
#endif