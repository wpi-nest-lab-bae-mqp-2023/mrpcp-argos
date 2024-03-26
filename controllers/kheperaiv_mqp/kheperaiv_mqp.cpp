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
   m_pcProximity = GetSensor  <CCI_KheperaIVProximitySensor>("kheperaiv_proximity");

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
    m_eState = STATE_ROTATING;

    c = 0;
    subtour_idc = 0;
    if(path_arr.empty()){ return; }
    goal_pos.SetX(path_arr[subtour_idc][c][0]);
    goal_pos.SetY(path_arr[subtour_idc][c][1]);
}

/****************************************/
/****************************************/

void CKheperaIVMQP::ControlStep() {
    /* Get readings from proximity sensor */
    const CCI_KheperaIVProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    /* Sum them together */
    CVector2 cAccumulator;
    for(size_t i = 0; i < tProxReads.size(); ++i) {
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator /= tProxReads.size();

//    std::cout << "Angle" << cAccumulator.Angle() << "Length" << cAccumulator.Length() << std::endl;
    curr_pos = m_pcPosSens->GetReading().Position;
    quat = m_pcPosSens->GetReading().Orientation;
    quat.ToEulerAngles(yaw, temp1, temp2);

    double x_err = goal_pos.GetX() - curr_pos.GetX();
    double y_err = goal_pos.GetY() - curr_pos.GetY();
    dist_err = sqrt(pow(x_err, 2) + pow(y_err, 2));

    angle_err = atan2(y_err, x_err) - yaw.GetValue();
    if(angle_err >= M_PI) { angle_err -= 2 * M_PI; }
    if(angle_err < -M_PI) { angle_err += 2 * M_PI; }

    if(path_arr.empty()){ return; }

    switch(m_eState) {
     case STATE_ROTATING: {
         Rotate();
         break;
     }
     case STATE_DRIVE: {
         Drive();
         break;
     }
     case STATE_NEW_POINT: { //gets a new point from the .argos file -- currently set to just rotate through infinitely
       c += 1;
       if(c==path_arr[subtour_idc].size()){
         c = 0;
         subtour_idc += 1;
         if(subtour_idc==path_arr.size()){
             subtour_idc = 0;
         }
       }
       goal_pos.SetX(path_arr[subtour_idc][c][0]);
       goal_pos.SetY(path_arr[subtour_idc][c][1]);
       std::cout << "New Point:" << goal_pos.GetX() << "," << goal_pos.GetY() << std::endl;

       m_eState = STATE_ROTATING;
       break;
     }
     default: {
         std::cout << "We can't be here, there's a bug!" << std::endl;
     }
  }
}

//rotates to a set (x, y) position. yaw is the current orientation
void CKheperaIVMQP::Rotate(){
    if(abs(angle_err) > 0.02){
        double omega_eff = kp * angle_err - kd * prev_angle_err;
        ApplyTwist(0., omega_eff);
        prev_angle_err = angle_err;
    }
    else{
        ApplyTwist(0., 0.);
        m_eState = STATE_DRIVE;
    }
}

//drives a certain distance
void CKheperaIVMQP::Drive(){

    // If there, get new point
    if(dist_err < 0.05){
        ApplyTwist(0., 0.);
        m_eState = STATE_NEW_POINT;
        return;
    }
    // If not, drive there
    ApplyTwist(m_fWheelVelocity * 0.8, angle_err * 10.);
}

void CKheperaIVMQP::ApplyTwist(double v_eff, double omega_eff) {
    double left_wheel_vel = v_eff - (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    double right_wheel_vel = v_eff + (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    m_pcWheels->SetLinearVelocity(left_wheel_vel, right_wheel_vel);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CKheperaIVMQP, "kheperaiv_mqp_controller")
