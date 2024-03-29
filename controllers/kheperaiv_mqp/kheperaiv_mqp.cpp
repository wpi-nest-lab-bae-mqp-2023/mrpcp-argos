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
   maxRobotVelocity(2.5),
   maxRobotOmega(10.),
   vel_kp(1.),
   vel_ki(0.),
   vel_kd(0.),
   theta_kp(20.),
   theta_ki(0.),
   theta_kd(0.1),
   vel_ctrl(vel_kp, vel_ki, vel_kd, -maxRobotVelocity, maxRobotVelocity),
   theta_ctrl(theta_kp, theta_ki, theta_kd, -maxRobotOmega, maxRobotOmega)
   {}

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
   GetNodeAttributeOrDefault(t_node, "max_velocity", maxRobotVelocity, maxRobotVelocity);
   GetNodeAttributeOrDefault(t_node, "max_omega", maxRobotOmega, maxRobotOmega);
   GetNodeAttributeOrDefault(t_node, "vel_kp", vel_kp, vel_kp);
   GetNodeAttributeOrDefault(t_node, "vel_ki", vel_ki, vel_ki);
   GetNodeAttributeOrDefault(t_node, "vel_kd", vel_kd, vel_kd);
   GetNodeAttributeOrDefault(t_node, "theta_kp", theta_kp, theta_kp);
   GetNodeAttributeOrDefault(t_node, "theta_ki", theta_ki, theta_ki);
   GetNodeAttributeOrDefault(t_node, "theta_kd", theta_kd, theta_kd);

   vel_ctrl = PIDController(vel_kp, vel_ki, vel_kd, -maxRobotVelocity, maxRobotVelocity);
   theta_ctrl = PIDController(theta_kp, theta_ki, theta_kd, -maxRobotOmega, maxRobotOmega);

   Reset();
}

void CKheperaIVMQP::Reset(){
    m_eState = STATE_ROTATING;

    c = 0;
    subtour_idc = 0;
    if(path_arr.empty()){ return; }
    goal_pos.SetX(path_arr[subtour_idc][c][0]);
    goal_pos.SetY(path_arr[subtour_idc][c][1]);

    vel_ctrl.reset();
    theta_ctrl.reset();

    prev_pos_filled = false;
    prev_pos = CVector3();
}

/****************************************/
/****************************************/

void CKheperaIVMQP::ControlStep() {
    curr_pos = m_pcPosSens->GetReading().Position;
    if (prev_pos_filled) { curr_vel = (curr_pos - prev_pos) / 0.1; }
    quat = m_pcPosSens->GetReading().Orientation;
    quat.ToEulerAngles(yaw, temp1, temp2);

    double x_err = goal_pos.GetX() - curr_pos.GetX();
    double y_err = goal_pos.GetY() - curr_pos.GetY();
    dist_err = sqrt(pow(x_err, 2) + pow(y_err, 2));

    angle_err = atan2(y_err, x_err) - yaw.GetValue();
    if(angle_err >= M_PI) { angle_err -= 2 * M_PI; }
    if(angle_err < -M_PI) { angle_err += 2 * M_PI; }

    if(path_arr.empty()){ return; }

//    if (id == 0) { return; }

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
    prev_pos = curr_pos;
    prev_pos_filled = true;
}

//rotates to a set (x, y) position. yaw is the current orientation
void CKheperaIVMQP::Rotate(){
    if(abs(angle_err) > 0.02){
        double omega_eff = theta_ctrl.computeEffort(angle_err);
        ApplyTwist(0., omega_eff);
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

    /* Calculate f_r (repulsive force) */
    CVector2 f_r;
    const CCI_KheperaIVProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    for(size_t i = 0; i < tProxReads.size(); ++i) {
        f_r -= CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    f_r /= tProxReads.size();
    double d = KHEPERAIV_IR_SENSORS_RING_RANGE - f_r.Length();
//    std::cout << "d: " << d << std::endl;
    if (f_r.Length() > 0.) { f_r.Normalize(); }
    f_r.Rotate(CRadians(yaw.GetValue()));
//    double f_r_angle_err = atan2(f_r.GetY(), f_r.GetX());  // - yaw.GetValue();
//    if(f_r_angle_err >= M_PI) { f_r_angle_err -= 2 * M_PI; }
//    if(f_r_angle_err < -M_PI) { f_r_angle_err += 2 * M_PI; }
//    std::cout << "f_r angle_err: " << f_r_angle_err << std::endl;

    /* Calculate f_a (attractive force) */
    CVector2 f_a(goal_pos.GetX() - curr_pos.GetX(), goal_pos.GetY() - curr_pos.GetY());
    if (f_a.Length() > 0.) { f_a.Normalize(); }
//    double f_a_angle_err = atan2(f_a.GetY(), f_a.GetX()) - yaw.GetValue();
//    if(f_a_angle_err >= M_PI) { f_a_angle_err -= 2 * M_PI; }
//    if(f_a_angle_err < -M_PI) { f_a_angle_err += 2 * M_PI; }
//    std::cout << "f_a angle_err: " << f_a_angle_err << std::endl;

    double f = std::max(0., (-df/(d_max - d_min)) * (d - d_min) + df);
    CVector2 f_t = f_a + f * f_r;

    angle_err = atan2(f_t.GetY(), f_t.GetX()) - yaw.GetValue();
    if(angle_err >= M_PI) { angle_err -= 2 * M_PI; }
    if(angle_err < -M_PI) { angle_err += 2 * M_PI; }
//    std::cout << "f_t angle_err: " << angle_err << std::endl;
    double omega_eff = theta_ctrl.computeEffort(angle_err);

    double v_des_coeff = std::max(0., std::min(1., (d - d_min) / (d_max - d_min)));
    // Clip it based on distance left
    v_des_coeff = std::min(v_des_coeff, std::max(dist_err, KHEPERAIV_BASE_RADIUS) / (KHEPERAIV_BASE_RADIUS));
    double v_err = maxRobotVelocity * v_des_coeff - curr_vel.Length();
    if (f_r.Length() > 0. && (angle_err >= M_PI / 2. || angle_err <= -M_PI / 2.)) { v_err *= -1.; }
//    std::cout << "v_err: " << v_err << std::endl;

    double vel_eff = vel_ctrl.computeEffort(v_err);
//    std::cout << "vel_eff: " << vel_eff << std::endl;

    ApplyTwist(vel_eff, omega_eff);
}

void CKheperaIVMQP::ApplyTwist(double v_eff, double omega_eff) {
    double left_wheel_vel = v_eff - (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    double right_wheel_vel = v_eff + (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    m_pcWheels->SetLinearVelocity(left_wheel_vel, right_wheel_vel);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CKheperaIVMQP, "kheperaiv_mqp_controller")
