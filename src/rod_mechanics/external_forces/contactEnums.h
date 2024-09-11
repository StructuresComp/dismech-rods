#ifndef CONTACTENUMS_H
#define CONTACTENUMS_H


enum ConstraintType {
    PointToPoint=0,
    PointToEdge,
    EdgeToEdge
};

enum ContactPiecewise {
    NonPenetrated=0,
    Penetrated
};

enum FrictionType {
    ZeroVel=0,
    Sliding,
    Sticking
};

#endif
