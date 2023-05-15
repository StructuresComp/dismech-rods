 /**
  * pwmPeripheral.h
  * Declarations for the class pwmPeripheral.
  * The "Peripheral" implementation of a PWM block is motivated by Nordic's
  * physical peripheral on a microcontroller. 
  * Updates to frequency or period do not get applied until the start of the following cycle
  * (c.f., bit bang.)
  * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
  *
  */

#ifndef PWM_PERIPHERAL_H
#define PWM_PERIPHERAL_H

// Some constants used throughout this program
#include "../global_const.h"

#include <iostream> // i/o
#include <exception> // for error handling


class pwmPeripheral
{
    public:

    /**
     * Construct a pwmPeripheral.
     * Requires an initial period and duty cycle.
     * @param m_period, expressed in seconds
     * @param m_duty, duty cycle expressed as a percent.
     * @throws runtime error if duty invalid
     */
    pwmPeripheral(double m_period, double m_duty);
    ~pwmPeripheral();

    /**
     * Enable or disable the pwm output.
     * @param onoff, 1 = on, 0 = off.
     */
    void setOnOff(bool onoff);

    // and a getter for state.
    bool isOn();

    /**
     * Change the control parameters.
     */
    void setPeriod(double m_period);
    void setDutyCycle(double m_duty);

    /**
     * Return the current state of the PWM.
     * Abstracting away the magnitude, just 0 or 1.
     * @return highlow, the pulse output by the pwm.
     */
    int getHighLow();

    /**
     * It's required to timestep the PWM so it can update its local variables.
     */
    void updateTimestep(double dt);

    private:

    // Helper functions to validate various things passed in.
    // @throws runtime errors as appropriate
    void validatePeriod(double per);
    void validateDuty(double dut);

    // a small initialization function. Used both when turning on and when completing a cycle,
    // both cases where a new cycle starts.
    void reinit();

    // pwm parameters
    double period;
    double duty;

    // This implementation does not update its parameters until the next cycle, so what we'll do
    // is just store the next value to be used, which will be what the set functions actually modify.
    double next_period;
    double next_duty;

    // This PWM block knows if it should output something or nothing
    bool is_on;

    // Track the total time since the simulation began.
    double current_time;
    // and a local variable for the dt between last period start and current time, for fewer computations
    double dt_current_period;

    // Tracking the duty cycle requires the last time a cycle started
    // has no meaning if the pwm is off!
    double last_start_time;

    // it's useful to store a local variable which determines when to be low during this cycle
    // microcontrollers call this a capture/compare value
    // has no meaining if the pwm is off!
    double cc;
};

#endif // PWM_PERIPHERAL_H