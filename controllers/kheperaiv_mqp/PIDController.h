//
// Created by Yaşar İdikut on 3/26/24.
//

#ifndef MRPCP_ARGOS_PIDCONTROLLER_H
#define MRPCP_ARGOS_PIDCONTROLLER_H

#include <algorithm>


class PIDController {
private:
    double Kp, Ki, Kd;

    double prevError = 0.;
    double sumError = 0.;
    double minEffort = 0.;
    double maxEffort = 0.;
    double minAbsEffort = 0.;

public:
    PIDController(double p, double i, double d, double minEff, double maxEff, double minAbsEff=0.) : Kp(p), Ki(i), Kd(d), minEffort(minEff), maxEffort(maxEff), minAbsEffort(minAbsEff) {}
    double computeEffort(double err);
    void reset() { sumError = 0.; }
};


#endif //MRPCP_ARGOS_PIDCONTROLLER_H
