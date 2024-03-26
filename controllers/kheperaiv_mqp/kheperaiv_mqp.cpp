/* Include the controller definition */
#include "kheperaiv_mqp.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <sstream>
#include <list>
#include <utility>


CKheperaIVMQP::CKheperaIVMQP() :
   m_pcWheels(NULL),
   m_pcPosSens(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/
void CKheperaIVMQP::SetPath(std::vector<std::vector<std::vector<float>>> path_arrki) {
    path_arr = std::move(path_arrki);
    Reset();
}


void CKheperaIVMQP::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><kheperaiv_diffusion><actuators> and
    * <controllers><kheperaiv_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_kheperaivProximitySensor      >("kheperaiv_proximity");

   m_pcPosSens   = GetSensor  <CCI_PositioningSensor        >("positioning");
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   //m_cOutput.open("output.txt", std::ios::app);
   //m_cOutput << m_pcPosSens->GetReading().Position;

   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   Reset();
}

void CKheperaIVMQP::Reset(){

    for (int subtouri = 0; subtouri < path_arr.size(); ++subtouri) {
        std::cout << "\tSubtour #" << subtouri << std::endl;
        for (int pointi = 0; pointi < path_arr[subtouri].size(); ++pointi) {
            std::cout << "\t\tPoint: [" << path_arr[subtouri][pointi][0] << ", " << path_arr[subtouri][pointi][1] << "]" << std::endl;
        }
    }

    m_eState = STATE_ROTATING;
}



/****************************************/
/****************************************/

void CKheperaIVMQP::ControlStep() {
   /* Get readings from proximity sensor */
//   const CCI_kheperaivProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
//   CVector2 cAccumulator;
//   for(size_t i = 0; i < tProxReads.size(); ++i) {
//      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
//   }
//   cAccumulator /= tProxReads.size();

   pos = m_pcPosSens->GetReading().Position;
   quat = m_pcPosSens->GetReading().Orientation;

   /* If the angle of the vecUInt32tor is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
    if(path_arr.empty()){
        return;
    }

    std::cout << "..." << std::endl;

    switch(m_eState) {
     case STATE_ROTATING: {
       quat.ToEulerAngles(yaw, temp1, temp2);
       Rotate(path_arr[subtour_idc][c][0]-pos[0], path_arr[subtour_idc][c][1]-pos[1], yaw);
       break;
     }
     case STATE_DRIVE: {
       double dist = sqrt(pow(path_arr[subtour_idc][c][0]-pos[0],2)+pow(path_arr[subtour_idc][c][1]-pos[1],2));
       Drive(dist);
       break;
     }
     case STATE_NEW_POINT: { //gets a new point from the .argos file -- currently set to just rotate through infinitely
       c += 1;
       if(c==path_arr[subtour_idc].size()){
         c = 0;
         subtour_idc += 1;
       }
         std::cout << "new point:" << path_arr[subtour_idc][c][0] << "," << path_arr[subtour_idc][c][1] << std::endl;

       m_eState = STATE_ROTATING;
       break;
     }
     default: {
         std::cout << "We can't be here, there's a bug!" << std::endl;
     }
  }
}

//rotates to a set (x, y) position. yaw is the current orientation
void CKheperaIVMQP::Rotate(double x, double y, argos::CRadians yaw){
    angleerr =  atan2(y, x) - yaw.GetValue();
    while(angleerr >= pi){
      angleerr -= 2*pi;
    }
    while(angleerr <= -pi){
      angleerr += 2*pi;
    }

    if(abs(angleerr) > 0.02){
      ideal_speed = std::max(std::min(minimum_speed,abs(kp*angleerr - kd*prevangleerr)), maximum_speed);
      if(angleerr<0){
        m_pcWheels->SetLinearVelocity(ideal_speed, -ideal_speed);
      }
      else{
        m_pcWheels->SetLinearVelocity(-ideal_speed, ideal_speed);
      }
      prevangleerr = angleerr;
    }
    else{
      m_pcWheels->SetLinearVelocity(0, 0);
      m_eState = STATE_DRIVE;
    }
}

//drives a certain distance
void CKheperaIVMQP::Drive(double distance){
    if(distance > 0.05 || distance < -0.05){
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }
    else {
      m_pcWheels->SetLinearVelocity(0, 0);
      m_eState = STATE_NEW_POINT;
    }
}
/*
void CKheperaIVMQP::Rotate(double distance){

}
*/

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CKheperaIVMQP, "kheperaiv_mqp_controller")
