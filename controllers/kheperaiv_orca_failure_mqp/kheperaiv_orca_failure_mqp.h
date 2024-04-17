#ifndef KHEPERA_ORCA_FAILURE_MQP_H
#define KHEPERA_ORCA_FAILURE_MQP_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_measures.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include "controllers/kheperaiv_mqp/PIDController.h"

#include "RVO.h"
#include "controllers/kheperaiv_orca_mqp/Filter.h"


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CKheperaIVORCAFailureMQP : public CCI_Controller {

public:
    CKheperaIVORCAFailureMQP();
    virtual ~CKheperaIVORCAFailureMQP() {}

    virtual void Init(TConfigurationNode& t_node);
    virtual void ControlStep();

    virtual void Reset();
    virtual void Destroy() {}

    unsigned long id;

    CVector2 curr_pos;
    CVector2 curr_vel;
    CVector2 goal_pos;
    CVector2 depot_pos;

    struct NORCADataBytes {
        double curr_pos_x;
        double curr_pos_y;
        double curr_vel_x;
        double curr_vel_y;
    };
    class NORCAData {
    public:
        CVector2 curr_pos;
        CVector2 curr_vel;
        NORCAData(): curr_pos(CVector2()), curr_vel(CVector2()) {}
        NORCAData(CVector2 curr_pos, CVector2 curr_vel) :
            curr_pos(curr_pos), curr_vel(curr_vel) {}
        explicit NORCAData(NORCADataBytes nVelVecBytes) {
            curr_pos = CVector2(nVelVecBytes.curr_pos_x, nVelVecBytes.curr_pos_y);
            curr_vel = CVector2(nVelVecBytes.curr_vel_x, nVelVecBytes.curr_vel_y);
        }
        NORCADataBytes GetBytes() {
            return NORCADataBytes{
                    curr_pos.GetX(),
                    curr_pos.GetY(),
                    curr_vel.GetX(),
                    curr_vel.GetY(),
            };
        }
        static RVO::Vector2 ARGOStoRVOVec(CVector2 arg_vec) { return {(float)arg_vec.GetX(), (float)arg_vec.GetY()}; }
        static CVector2 RVOtoARGOSVec(RVO::Vector2 rvo_vec) { return {(double)rvo_vec.x(), (double)rvo_vec.y()}; }
        RVO::Vector2 GetCurrPos(){ return ARGOStoRVOVec(curr_pos); }
        RVO::Vector2 GetCurrVel(){ return ARGOStoRVOVec(curr_vel); }
    };
//    std::vector<NORCAData> nORCADatas = std::vector<NORCAData>();

    CVector2 orcaVec;
    std::vector<std::vector<RVO::Vector2>> obstacles = std::vector<std::vector<RVO::Vector2>>();
    double rab_range;

    std::vector<std::vector<std::vector<double>>> path_arr;

    virtual void SetPath(std::vector<std::vector<std::vector<double>>> path_arrki);
    bool is_turn_to_startup_depot = false;
    bool did_leave_from_startup_depot = false;

    double fr = 0.;
    unsigned int since_failed_counter = 0;
    unsigned int since_deadlock_counter = 0;
    unsigned int since_non_deadlock_counter = 0;

private:
    void UpdateVelocityVector(CCI_DifferentialSteeringSensor::SReading pcWheelsSReading);
    void BroadcastORCA();
    void DriveORCA(CVector2 orca_goal_pos);
    static NORCAData GetORCAData(CCI_RangeAndBearingSensor::SPacket sPacket);

    void ResetSim();

    CRadians yaw, temp1, temp2;

    /* Pointer to the differential steering actuator and sensor */
    CCI_DifferentialSteeringActuator* m_pcWheelsA;
    CCI_DifferentialSteeringSensor* m_pcWheelsS;
    CCI_PositioningSensor* m_pcPosSens;
    /* Pointer to the LEDs actuator */
    CCI_LEDsActuator* m_pcLEDs;

    /* Pointer to the range and bearing actuator */
    CCI_RangeAndBearingActuator*  m_pcRABA;
    /* Pointer to the range and bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABS;

    RVO::RVOSimulator *simulator = new RVO::RVOSimulator();

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
    void ApplyORCA(CVector2 VelVec);
    void ApplyTwist(double v_eff, double omega_eff);

    AveragingFilter curr_vel_x_filter;
    AveragingFilter curr_vel_y_filter;
    AveragingFilter deadlock_detection_filter;

    /* The three possible states in which the controller can be */
    enum EState {
        STATE_NEW_POINT = 0,
        STATE_DRIVE,
        STATE_DEADLOCK,
        STATE_FAILURE,
    };
    EState m_eState;
    EState m_eState_prev;

    unsigned long subtour_idx;
    unsigned long node_idx;

    double dist_err = 0.;
    double angle_err = 0.;

    bool wasRotating = false;
    double drivingRotationTolerance = M_PI / 12.;  // M_PI/12. = +- 15. degrees
    double rotatingRotationTolerance = M_PI / 48.;  // M_PI/48. = +- 3.75 degrees

    CVector2 orcaNeighborsCentroid = CVector2();
    double orcaTimeHorizon = 2.;
};

#endif
