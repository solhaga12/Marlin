#include "torch_height_control.h"
#include "../../module/stepper.h"

#define PAUSE_TIMER4  TCCR4B = _BV(WGM43);
#define RESUME_TIMER4 TCCR4B = _BV(WGM43) | _BV(CS41) | _BV(CS40);

THCState TorchHeightController::state = Disabled;
bool TorchHeightController::countingUp = true;
int32_t TorchHeightController::targetSpeed = 0;
int16_t TorchHeightController::speed = 0;
int16_t TorchHeightController::maxAcc = 0;
int8_t TorchHeightController::direction = 1;
uint16_t TorchHeightController::maxStoppingDistance = 0xFFFF;
long TorchHeightController::zTopPosition = 0;
long TorchHeightController::zBottomPosition = 0;

int16_t TorchHeightController::newTargetSpeed = 25000;
int16_t TorchHeightController::counter = 0;

//----------------------------------------------------------------------------//
void TorchHeightController::init()
{

  zTopPosition = 10; // sw_endstop_max[Z_AXIS] * planner.axis_steps_per_mm[Z_AXIS]; TODO What to use here?
  zBottomPosition = 0;

  // Set maximal period (considered as speed = 0)
  ICR4 = 0xFFFF;

  // Phase correct PWM mode with ICR4 on TOP, prescaler 64
  TCCR4A = _BV(WGM41);
  TCCR4B = _BV(WGM43);

  // Init counter to maxval to mean last Z step is at least this old
  TCNT4 = 0xFFFF;

  // allow interrupts
  TIMSK4 = _BV(TOIE4) | _BV(ICF4);
}
//----------------------------------------------------------------------------//
void TorchHeightController::enable()
{
  stepper.leave_control_on(Z_AXIS);
  Z_ENABLE_WRITE(Z_ENABLE_ON);

  resetPID();

  state = Enabled;
}
//----------------------------------------------------------------------------//
void TorchHeightController::disable()
{
  targetSpeed = 0;
  if(state == Enabled)
    state = Disabling;
}
//----------------------------------------------------------------------------//
THCState TorchHeightController::getState()
{
  return state;
}
//----------------------------------------------------------------------------//
void TorchHeightController::update()
{
  // read the set and read voltage??
  int32_t voltage_mv = ADS1015_device.read();
  if(state == Enabled)
  {
    //PID--------------//
    if(counter == 200)
    {
      counter = 0;
      newTargetSpeed = -newTargetSpeed;
    }
    else
    {
      counter++;
    }
    targetSpeed = newTargetSpeed;
    //--------------//
  }

  //THC is disabled or disabling
  if(state != Enabled)
  {
    targetSpeed = 0;
    if(state == Disabling && speed == 0)
    {
      // Z have stopped, give back Z control to stepper class
      planner.set_z_position_step(stepper.position(Z_AXIS));
      stepper.take_control_on(Z_AXIS);
      state = Disabled;
    }
  }
  else // THC is enabled
  {
    // check for software endstop overrun
    long z_pos = stepper.position(Z_AXIS);
    if(z_pos > zTopPosition || z_pos < zBottomPosition)
    {
      kill("Stop: Z overrun.");
    }

  }

  int32_t acc = targetSpeed - speed;

  // don't touch anything, speed is fine
  if(acc == 0)
    return;

  // clip acceleration to maxAcc
  if(acc > maxAcc)
    acc = maxAcc;
  else if(acc < -maxAcc)
    acc = -maxAcc;

  // apply acceleration
  speed = speed + acc;

  int16_t maxSpeed = PLASMA_MAX_THC_STEP_S;
  if(speed > maxSpeed)
    speed = maxSpeed;
  else if(speed < -maxSpeed)
    speed = -maxSpeed;

  // compute step direction and frequency
  uint16_t frequency;
  if(speed >= 0)
  {
    frequency = speed;
    direction = 1;
  }
  else
  {
    frequency = -speed;
    direction = -1;
  }

  // compute interrupts period for step generation
  uint16_t period;
  if(frequency == 0)
    period = 0xFFFF;
  else
    period = min(250000 / frequency, 0xFFFF);

  PAUSE_TIMER4;
  Z_DIR_WRITE(INVERT_Z_DIR ^ (direction > 0));
  uint16_t elapsed = countingUp ? TCNT4 : ICR4 - TCNT4;
  uint16_t rest = elapsed > period ? 0 : period - elapsed;
  ICR4 = period;
  TCNT4 = countingUp ? period - rest : rest;
  RESUME_TIMER4;
}
//----------------------------------------------------------------------------//
void TorchHeightController::setmaxAccStepS2(unsigned long maxAcc)
{
  // store acceleration is milliseconds as update will be called at 1kHz
  maxAcc = maxAcc / 1000;

  unsigned long max_freq = PLASMA_MAX_THC_STEP_S;
  maxStoppingDistance = pow(maxFreq, 2) / maxAcc;
}
//----------------------------------------------------------------------------//
void TorchHeightController::resetPID()
{
  newTargetSpeed = 25000;
  counter = 100;
}
//----------------------------------------------------------------------------//
ISR(TIMER4_OVF_vect){ TorchHeightController::ovf_isr(); }
void TorchHeightController::ovf_isr()
{
  if(ICR4 == 0xFFFF)
  {
    PAUSE_TIMER4;
  }
  else
  {
    Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
    countingUp = true;
    stepper.shift_z_position(direction);
    Z_STEP_WRITE(INVERT_Z_STEP_PIN);
  }
}
//----------------------------------------------------------------------------//
ISR(TIMER4_CAPT_vect){ TorchHeightController::capt_isr(); }
void TorchHeightController::capt_isr()
{
  if(ICR4 == 0xFFFF)
  {
    PAUSE_TIMER4;
  }
  else
  {
    Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
    countingUp = false;
    stepper.shift_z_position(direction);
    Z_STEP_WRITE(INVERT_Z_STEP_PIN);
  }
}
//----------------------------------------------------------------------------//
