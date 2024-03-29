/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef kheperaiv_XML_H
#define kheperaiv_XML_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_proximity_sensor.h>

#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_measures.h>

#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include <fstream>

#include <cmath>

#include <string>

#include <sstream>

#include "PIDController.h"


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotCollisionHandling : public CCI_Controller {

public:

   /* Class constructor. */
   CFootBotCollisionHandling();

   /* Class destructor. */
   virtual ~CFootBotCollisionHandling() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><kheperaiv_mqp_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   virtual void Rotate();

   virtual void Drive();

   bool RotateToCircle(argos::CRadians desiredAngle, argos::CRadians yaw);

   void Circle(double radius);

   virtual void ApplyTwist(double v_eff, double omega_eff);

   std::vector<std::vector<std::vector<float>>> path_arr;
   virtual void SetPath(std::vector<std::vector<std::vector<float>>> path_arrki);

   unsigned long id;
   bool wait;

   double depot_x;
   double depot_y;


private:
  /* The three possible states in which the controller can be */
  enum EState {
    STATE_ROTATING = 0,
    STATE_DRIVE,
    STATE_NEW_POINT,
    AT_DEPOT
  };

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;

   CCI_KheperaIVProximitySensor* m_pcProximity;

   CCI_PositioningSensor* m_pcPosSens;

   CRadians ogYaw;

   std::string m_strOutput;
   std::ofstream m_cOutput;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><kheperaiv_mqp_controller> section.
    */

   /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
   CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
   Real m_fDelta;
   /* Wheel speed. */
   Real maxRobotVelocity;
   Real maxRobotOmega;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> m_cGoStraightAngleRange;

   CVector3 curr_pos;
   CVector3 prev_pos;
   bool prev_pos_filled = false;
   CVector3 curr_vel;
   CVector3 goal_pos;
   CQuaternion quat;
   CRadians yaw, temp1, temp2;

   // PID values
   double vel_kp;
   double vel_ki;
   double vel_kd;
   double theta_kp;
   double theta_ki;
   double theta_kd;

   double ideal_speed;

   double wheelbase = 0.14;

   CVector3 pos;

   double dist_err = 0.;
   double angle_err = 0.;
   double prev_angle_err = 0.;

   int c = 0;
   int subtour_idc = 0;
   EState m_eState;

   double kp = 5;
   double kd = 1;
   double minimum_speed = 4;
   double maximum_speed = 0;

   double angleerr = 0;
   double prevangleerr = 0;

   PIDController vel_ctrl;
   PIDController theta_ctrl;

   double df = 10.;
   double d_min = 0.05;
   double d_max = 0.125;

};

#endif
