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

#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include <fstream>

#include <cmath>

#include <string>

#include <sstream>
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CKheperaIVMQP : public CCI_Controller {

public:

   /* Class constructor. */
   CKheperaIVMQP();

   /* Class destructor. */
   virtual ~CKheperaIVMQP() {}

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

   virtual void Rotate(double x, double y, argos::CRadians yaw);

   virtual void Drive(double distance);

    std::vector<std::vector<std::vector<float>>> path_arr;
    virtual void SetPath(std::vector<std::vector<std::vector<float>>> path_arrki);

private:
  /* The three possible states in which the controller can be */
  enum EState {
    STATE_ROTATING = 0,
    STATE_DRIVE,
    STATE_NEW_POINT
  };

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   CCI_KheperaIVProximitySensor* m_pcProximity;

   CCI_PositioningSensor* m_pcPosSens;


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
   Real m_fWheelVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> m_cGoStraightAngleRange;

   CVector3 pos;
   CQuaternion quat;
   CRadians yaw, temp1, temp2;

   //pid values for rotating, could be tuned
   double kp = 5;
   double kd = 1;
   double minimum_speed = 4;
   double maximum_speed = 0;

   double ideal_speed;

   double angleerr, prevangleerr = 0;

   int c = 0;
   int subtour_idc = 0;
   std::string path;
   EState m_eState;
   double pi = 3.1415926;


};

#endif
