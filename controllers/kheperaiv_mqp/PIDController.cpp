//
// Created by Yaşar İdikut on 3/26/24.
//

#include "PIDController.h"

double PIDController::computeEffort(double err)
{
    sumError += err;

    if (Kp * err + Ki * sumError + Kd * (err - prevError) > maxEffort)
    {
        sumError -= err;
    }

    if (Kp * err + Ki * sumError + Kd * (err - prevError) < minEffort)
    {
        sumError -= err;
    }

    prevError = std::max(minEffort, std::min(maxEffort, Kp * err + Ki * sumError + Kd * (err - prevError)));
    if (std::abs(prevError) < minAbsEffort) {
        prevError = prevError > 0. ? minAbsEffort : -minAbsEffort;
    }

    return prevError;
}