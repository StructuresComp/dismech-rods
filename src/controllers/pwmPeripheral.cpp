/**
 * pwmPeripheral.cpp
 * 
 * Definitions for the class pwmPeripheral. 
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#include "pwmPeripheral.h"

// Constructor. See .h file
pwmPeripheral::pwmPeripheral(double m_period, double m_duty) : period(m_period), duty(m_duty), 
                            next_period(m_period), next_duty(m_duty), is_on(false), current_time(0.0), 
                            dt_current_period(0.0), last_start_time(0.0), cc(0.0)
{
    if( verbosity >= 2){
        std::cout << "Constructing a pwmPeripheral..." << std::endl;
    }
    // Some checking against invalid parameters
    validateDuty(m_duty);
    validatePeriod(m_period);
    // PWM defaults to off, so no need to call reinit in the constructor.
}

pwmPeripheral::~pwmPeripheral()
{
}

// simple error checking functions
void pwmPeripheral::validatePeriod(double per)
{
    // period can't be negative
    if( per < 0.0 ){
        throw std::invalid_argument("Error, period must be a nonnegative number!");
    }

    // TO-DO: is this necessary? how to use fmod and check for == 0.0?
    /*
    // Hard coded for now: the simulation runs at 1 msec, so period must be an interval of that.
    if( (per % 1e-3) != 0 ){
        throw std::runtime_error("Error! Period for the PWM peripheral is not an interval of timestep.");
    }
    */
}

void pwmPeripheral::validateDuty(double dut)
{
    // Elsewhere will threshold duty cycle onto intervals of timestep,
    // so here, just check that it's between 0 and 100 percent, i.e. [0.0 1.0].
    if(( dut < 0.0) || (dut > 1.0)){
        throw std::runtime_error("Error! Duty cycle for PWM peripheral must be between 0.0 and 1.0 (i.e., 0 and 100 percent.)");
    }
}

void pwmPeripheral::setOnOff(bool onoff)
{
    // three different behaviors:
    // 1) going from off to on
    if( onoff && !is_on ){
        // init the peripheral
        reinit();
    }
    // else if ( !onoff && is_on ){
        // 2) going from on to off
        // anything here? we should just be able to leave the registers as "whatever" since they'll be overwritten before use again
    // }
    // 3) if same as previous, no changes needed.

    // in any case, update the state.
    is_on = onoff;
}

bool pwmPeripheral::isOn()
{
    return is_on;
}

// Changing the control parameters will be for the "next" cycle.
void pwmPeripheral::setPeriod(double m_period)
{
    validatePeriod(m_period);
    next_period = m_period;
}

void pwmPeripheral::setDutyCycle(double m_duty)
{
    validateDuty(m_duty);
    next_duty = m_duty;
}

// The big one: updating the timestep updates various local variables for the next cycle, if needed
void pwmPeripheral::updateTimestep(double dt)
{
    current_time += dt;
    // If the PWM is on at this time,
    if(is_on){
        // progress through this cycle 
        dt_current_period = current_time - last_start_time;
        // If we've completed one period,
        if(dt_current_period >= period){
            reinit();
        }
    }
    // if not on, the values in dt_current_period and last_start_time should be treated as garbage.
}


void pwmPeripheral::reinit()
{
    // Read in the next set of parameters (think: register-y from microcontroller)
    period = next_period;
    duty = next_duty;
    // Recalculate the capture/control value based on period and duty cycle
    // TO-DO: floor this to an interval of simulation timestep!!
    cc = period * duty;
    // Reset the counter
    dt_current_period = 0.0;
    last_start_time = current_time;
}

// Getting the output of the PWM is a simple check against the cc "register".
// Starts "on" at the beginning of a period, turns off after duty cycle has ended.
int pwmPeripheral::getHighLow()
{
    // if not on, complain
    if (!is_on)
    {
        // throw std::runtime_error("Error, you can't query the state of a PWM that's off!");
        // a better behavior is to emulate the microcontroller, which would pull the pin low.
        return 0;
    }
    // else, return something.
    // counting inclusively means strict inequality. TO-DO: is this actually duty cycle then? Or off-by-one?
    int hilo = (dt_current_period < cc ? 1 : 0);
    return hilo;
}