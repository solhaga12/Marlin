/*
 * 200 V and 10 bits FSD, gives a resolution of 0,2 V/bit
 * Supposedly 1 V is 0,38 mm
 * That means, unfiltered, one bit is 76 um.
 *
 * The longer the distance, the higher the voltage.
 */



#ifndef TORCH_HEIGHT_CONTROL_H
#define TORCH_HEIGHT_CONTROL_H

#include "../../Marlin.h"
#include "../../core/enum.h"

class TorchHeightController {
  public:
    static void init();
    static void enable();
    static void disable();
    static THCState getState();
    static void setMaxAccStepS2(unsigned long);

    static void update();
    static void ovf_isr();
    static void capt_isr();

  private:
    static void resetPID();

    static THCState state;
    static bool countingUp;
    static int32_t targetSpeed;
    static int16_t speed;
    static int16_t maxAcc;
    static int8_t direction;
    static uint16_t maxStoppingDistance;
    static long zTopPosition;
    static long zBottomPosition;

    static int16_t newTargetSpeed;
    static int16_t counter;
};

extern TorchHeightController torchHeightController;

#endif
