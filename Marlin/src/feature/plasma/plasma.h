#ifndef PLASMA_H
#define PLASMA_H

#include "../../core/enum.h"

#define TURN_PLASMA_ON WRITE(PLASMA_START_PIN, HIGH);
#define TURN_PLASMA_OFF WRITE(PLASMA_START_PIN, LOW);
#define IS_PLASMA_TRANSFERRED READ(PLASMA_TRANSFER_PIN) ? LOW : HIGH

class Plasma {

  public:
    void init();
    bool start();
    void stop();
    void stop_after_move();
    void lock();
    void unlock();
    PlasmaState update();
    PlasmaState get_state();

  private:
    static PlasmaState state;
    static bool stop_pending;
};

extern Plasma plasmaManager;

#endif
