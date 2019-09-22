#include "../../Marlin.h"
#include "plasma.h"
#include "../../module/planner.h"

Plasma plasmaManager;

PlasmaState Plasma::state = Locked;
bool Plasma::stop_pending = false;
//----------------------------------------------------------------------------//
void Plasma::init()
{
	SET_OUTPUT(PLASMA_START_PIN);
	SET_OUTPUT(PLASMA_VD_UPDATES_PIN);
	SET_INPUT_PULLUP(PLASMA_TRANSFER_PIN);
  stop();
}
//----------------------------------------------------------------------------//
bool Plasma::start()
{
  CRITICAL_SECTION_START;
  if(state != Locked)
  {
    state = Ignition;
    TURN_PLASMA_ON
  }
  CRITICAL_SECTION_END;
  return state != Locked;
}
//----------------------------------------------------------------------------//
void Plasma::stop()
{
  if(state != Locked)
  {
    state = Off;
  }
  TURN_PLASMA_OFF
  stop_pending = false;
}
//----------------------------------------------------------------------------//
void Plasma::stop_after_move()
{
  stop_pending = true;
}
  //----------------------------------------------------------------------------//
void Plasma::lock()
{
  stop();
  state = Locked;
}
//----------------------------------------------------------------------------//
void Plasma::unlock()
{
  state = Off;
}
//----------------------------------------------------------------------------//
PlasmaState Plasma::update()
{
  static PlasmaState oldState = Off;

  if (oldState != state) {
    switch(state)
    {
      case Locked:
        SERIAL_ECHOLN("Plasma state: Locked");
        break;
      case Off:
        SERIAL_ECHOLN("Plasma state: Off");
        break;
      case Ignition:
        SERIAL_ECHOLN("Plasma state: Ignition");
        break;
      case Established:
        SERIAL_ECHOLN("Plasma state: Established");
        break;
      case Lost:
        SERIAL_ECHOLN("Plasma state: Lost");
        break;
    }
    oldState = state;
  }

  switch(state)
  {
    case Locked:
      break;
    case Off:
      break;
    case Ignition:
      if(IS_PLASMA_TRANSFERRED)
        state = Established;
      break;
    case Established:
      if(!IS_PLASMA_TRANSFERRED)
      {
        stop();
        state = Lost;
      }
      else if(stop_pending && !planner.has_blocks_queued())
      {
        stop();
      }
      break;
    case Lost:
      break;
  }
  return state;
}
//----------------------------------------------------------------------------//
PlasmaState Plasma::get_state()
{
  return state;
}
//----------------------------------------------------------------------------//
