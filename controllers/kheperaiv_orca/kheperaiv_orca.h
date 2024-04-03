#ifndef KHEPERA_ORCA_H
#define KHEPERA_ORCA_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_measures.h>

#include "RVO.h"

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CKheperaIVORCA : public CCI_Controller {

public:
    CKheperaIVORCA();
    virtual ~CKheperaIVORCA() {}

    virtual void Init(TConfigurationNode& t_node);
    virtual void ControlStep();

    virtual void Reset();
    virtual void Destroy() {}

    unsigned long id;

    CVector2 curr_pos;
    CVector2 curr_vel;
    CVector2 goal_pos;

    struct NVelVecBytes {
        bool hasFailed;
//        double pos_x;
//        double pos_y;
        double vel_x;
        double vel_y;
    };
    class NVelVec {
    public:
        bool hasFailed;
        CVector2 pos;
        CVector2 vel;
        NVelVec(): hasFailed(true), pos(CVector2()), vel(CVector2()) {}
        NVelVec(bool hasFailed, CVector2 pos, CVector2 vel): hasFailed(hasFailed), pos(pos), vel(vel) {}
    };
    NVelVec curr_vel_vec = NVelVec();
    std::vector<NVelVec> n_vel_vecs = std::vector<NVelVec>();

private:
    void UpdateVelocityVector(CCI_DifferentialSteeringSensor::SReading pcWheelsSReading);
    void BroadcastVelocityVector();
    NVelVec GetVelocityVector(CCI_RangeAndBearingSensor::SPacket sPacket);

    CRadians yaw, temp1, temp2;

    /* Pointer to the differential steering actuator and sensor */
    CCI_DifferentialSteeringActuator* m_pcWheelsA;
    CCI_DifferentialSteeringSensor* m_pcWheelsS;
    CCI_PositioningSensor* m_pcPosSens;

    /* Pointer to the range and bearing actuator */
    CCI_RangeAndBearingActuator*  m_pcRABA;
    /* Pointer to the range and bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABS;
};

#endif
