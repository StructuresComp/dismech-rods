#ifndef CONTACT_ENUMS_H
#define CONTACT_ENUMS_H

enum ConstraintType {
    POINT_TO_POINT = 0,
    POINT_TO_EDGE,
    EDGE_TO_EDGE
};

enum ContactPiecewise {
    NON_PENETRATED = 0,
    PENETRATED
};

enum FrictionType {
    ZERO_VEL = 0,
    SLIDING,
    STICKING
};

#endif
