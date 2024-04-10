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
#include <argos3/core/utility/math/rng.h>

#include <controllers/kheperaiv_mqp/PIDController.h>


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

    struct NORCADataBytes {
        bool hasFailed;
        double curr_pos_x;
        double curr_pos_y;
        double curr_pos_h; // heading/yaw
        double goal_pos_x;
        double goal_pos_y;
    };
    class NORCAData {
    public:
        bool hasFailed;
        CVector2 curr_pos;
        CRadians curr_pos_h;
        CVector2 goal_pos;
        NORCAData(): hasFailed(true), curr_pos(CVector2()), curr_pos_h(0.), goal_pos(CVector2()) {}
        NORCAData(bool hasFailed, CVector2 curr_pos, CRadians curr_pos_h, CVector2 goal_pos) :
            hasFailed(hasFailed), curr_pos(curr_pos), curr_pos_h(curr_pos_h), goal_pos(goal_pos) {}
        explicit NORCAData(NORCADataBytes nVelVecBytes) {
            hasFailed = nVelVecBytes.hasFailed;
            curr_pos = CVector2(nVelVecBytes.curr_pos_x, nVelVecBytes.curr_pos_y);
            curr_pos_h = CRadians(nVelVecBytes.curr_pos_h);
            goal_pos = CVector2(nVelVecBytes.goal_pos_x, nVelVecBytes.goal_pos_y);
        }
        NORCADataBytes GetBytes() {
            return NORCADataBytes{
                    hasFailed,
                    curr_pos.GetX(),
                    curr_pos.GetY(),
                    curr_pos_h.GetValue(),
                    goal_pos.GetX(),
                    goal_pos.GetY(),
            };
        }
        static RVO::Vector2 ARGOStoRVOVec(CVector2 arg_vec) { return {(float)arg_vec.GetX(), (float)arg_vec.GetY()}; }
        static CVector2 RVOtoARGOSVec(RVO::Vector2 rvo_vec) { return {(double)rvo_vec.x(), (double)rvo_vec.y()}; }
        RVO::Vector2 GetCurrPos(){ return ARGOStoRVOVec(curr_pos); }
        RVO::Vector2 GetGoalPos(){ return ARGOStoRVOVec(goal_pos); }
    };
//    std::vector<NORCAData> nORCADatas = std::vector<NORCAData>();

    CVector2 orcaVec;

private:
    void UpdateVelocityVector(CCI_DifferentialSteeringSensor::SReading pcWheelsSReading);
    void BroadcastORCA();
    static NORCAData GetORCAData(CCI_RangeAndBearingSensor::SPacket sPacket);

    void ResetSim();

    CRadians yaw, temp1, temp2;

    /* Pointer to the differential steering actuator and sensor */
    CCI_DifferentialSteeringActuator* m_pcWheelsA;
    CCI_DifferentialSteeringSensor* m_pcWheelsS;
    CCI_PositioningSensor* m_pcPosSens;

    /* Pointer to the range and bearing actuator */
    CCI_RangeAndBearingActuator*  m_pcRABA;
    /* Pointer to the range and bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABS;

    RVO::RVOSimulator *simulator;

    CRandom::CRNG *m_pcRNG;

    /* Wheel speed. */
    Real maxRobotVelocity;
    Real maxRobotOmega;
    // PID values
    double vel_kp;
    double vel_ki;
    double vel_kd;
    double theta_kp;
    double theta_ki;
    double theta_kd;
    PIDController vel_ctrl;
    PIDController theta_ctrl;

    virtual void ApplyTwist(double v_eff, double omega_eff);

};

#endif
