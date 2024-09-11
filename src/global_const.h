/**
 * global_const.h
 *
 * Handy file for declaring constants used across the entire simRollingStar program.
 *
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef GLOBAL_CONST_H
#define GLOBAL_CONST_H

/**
 * Debugging verbosity level.
 * 0 = no output (original code),
 * 1 = some output (intermittent, not every timestep)
 * 2 = output for every timestep.
 */
// const int verbosity = 1;
// We'll store verbosity from the input options file. Global!
extern int verbosity;


#endif // GLOBAL_CONST_H
