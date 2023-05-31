/**
 * rodEmptyController.h
 * 
 * Declarations for the concrete class rodEmptyController.
 * Actuation always off.
 * 
 * Copyright 2020 Andrew P. Sabelhaus and Soft Machines Lab at CMU
 */

#ifndef ROD_EMPTY_CONTROLLER_H
#define ROD_EMPTY_CONTROLLER_H

// everything comes from parent
#include "rodController.h"

// Need Eigen here for elasticRod interactions
#include "../eigenIncludes.h"

class rodEmptyController : public rodController
{

    public:

    /**
     * Constructor and destructor just call parents.
     */
    rodEmptyController(int numAct);
    ~rodEmptyController();

    /**
     * Calculate the desired control input, u(x). Always zero.
     * @param rod_p pointer to an elasticRod from which state feedback will occur.
     */
    std::vector<int> getU(shared_ptr<elasticRod> rod_p);

};

#endif //ROD_EMPTY_CONTROLLER_H