/* Include the controller definition */
#include "collision_mqp.h"
/* Function definitions for XML parsing */

#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <argos3/core/utility/logging/argos_log.h>

#include <sstream>
#include <list>
#include <utility>


CFootBotCollisionHandling::CFootBotCollisionHandling() :
   m_pcRNG(NULL),
   m_pcWheels(NULL),
   m_pcPosSens(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   maxRobotVelocity(2.5),
   maxRobotOmega(10.),
   m_fWheelVelocity(2.5f),
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
void CFootBotCollisionHandling::SetPath(std::vector<std::vector<std::vector<float>>> path_arrki) {
    path_arr = std::move(path_arrki);
    Reset();
}


void CFootBotCollisionHandling::Init(TConfigurationNode& t_node) {
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

void CFootBotCollisionHandling::Reset(){
    m_eState = GOING_TO_DEPOT;

    c = 0;
    subtour_idc = 0;

    if(path_arr.empty()){ return; }
    goal_pos.SetX(path_arr[subtour_idc][c][0]);
    goal_pos.SetY(path_arr[subtour_idc][c][1]);

    vel_ctrl.reset();
    theta_ctrl.reset();

    prev_pos_filled = false;
    prev_pos = CVector3();

    //m_pcRNG = argos::CRandom::CreateRNG(std::to_string(id));
    m_pcRNG = argos::CRandom::CreateRNG("argos");
    m_pcRNG->SetSeed(id);
}

/****************************************/
/****************************************/

void CFootBotCollisionHandling::ControlStep() {
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

    switch(m_eState) {
      case STATE_GOING_TO_POINT: {
        RotateDriveRotate(false);
       break;
      }
      case GOING_TO_DEPOT: {
        goal_pos.SetX(depot_x-0.5);
        goal_pos.SetY(depot_y-0.5);

        RotateDriveRotate(true);

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


        if(goal_pos.GetX() == depot_x-0.5 && goal_pos.GetY() == depot_y-0.5){
          goal_pos.SetX(depot_x);
          goal_pos.SetY(depot_y);
        }

        while(goal_pos.GetX() == path_arr[subtour_idc][c][0] && goal_pos.GetY() == path_arr[subtour_idc][c][1]){
          c += 1;
          if(c==path_arr[subtour_idc].size()){
            c = 0;
          }
        }


        goal_pos.SetX(path_arr[subtour_idc][c][0]);
        goal_pos.SetY(path_arr[subtour_idc][c][1]);

        if(goal_pos.GetX() == depot_x && goal_pos.GetY() == depot_y){
          m_eState = GOING_TO_DEPOT;
        }
        else{
          m_eState = STATE_GOING_TO_POINT;
        }
        break;
      }
      case AT_DEPOT: {
        Circle(0.75);
        break;
      }
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
   }

    prev_pos = curr_pos;
    prev_pos_filled = true;
}

void CFootBotCollisionHandling::RotateDriveRotate(bool goingToDepot){
  if(finished_first_rot == false){
    finished_first_rot = Rotate();
  }
  else{ //when finished rotating, drive to desired position
      if(finished_drive == false){
        finished_drive = Drive();
      }
      else{ //when finished driving, rotate to desired angle
        if(goingToDepot && finished_second_rot == false){
          if(goingToDepot == true){

            //just making sure that the robot is aligned correctly to rotate
            goal_pos.SetX(depot_x-1);
            goal_pos.SetY(depot_y-1);

            angle_err = atan2(goal_pos.GetY() - curr_pos.GetY(), goal_pos.GetX() - curr_pos.GetX()) - yaw.GetValue();

            if(angle_err >= M_PI) { angle_err -= 2 * M_PI; }
            if(angle_err < -M_PI) { angle_err += 2 * M_PI; }


            finished_second_rot = Rotate();
          }
          else{
            finished_second_rot = Rotate();
          }
        }
        else {
          if(goingToDepot && finished_third_rot == false){
            finished_third_rot = RotateToCircle(ogYaw, yaw);
          }
          else{
            finished_drive = false;
            finished_first_rot = false;
            finished_second_rot = false;
            finished_third_rot = false;

            if(goingToDepot){
              ogPos.SetX(curr_pos.GetX());
              ogPos.SetY(curr_pos.GetY());
              m_eState = AT_DEPOT;
            }
            else{
              m_eState = STATE_NEW_POINT;
            }
          }
        }
      }
  }
}

//rotates to a set (x, y) position. yaw is the current orientation
bool CFootBotCollisionHandling::Rotate(){
    if(abs(angle_err) > 0.02){
        double omega_eff = theta_ctrl.computeEffort(angle_err);
        ApplyTwist(0., omega_eff);
        return false;
    }
    else{
        ApplyTwist(0., 0.);
        ogYaw.SetValue(yaw.GetValue() - 1.5708);
        return true;
    }
}

bool CFootBotCollisionHandling::RotateToCircle(argos::CRadians desiredAngle, argos::CRadians yaw){
    angleerr = desiredAngle.GetValue() - yaw.GetValue();
    while(angleerr >= M_PI){
      angleerr -= 2*M_PI;
    }
    while(angleerr <= -M_PI){
      angleerr += 2*M_PI;
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
      return true;
    }
    return false;
}

//drives a certain distance
bool CFootBotCollisionHandling::Drive(){
  double wait = 150;

  if(curr_pos.GetX() < depot_x+0.25 && curr_pos.GetY() < depot_y+0.25){
    wait = 400;
  }
  if(wait == true && wait_counter < wait){
    m_pcWheels->SetLinearVelocity(0, 0);

    wait_counter += 1;

    if(willCollide == true){
      LOGERR << "will collide" << std::endl;
        m_pcWheels->SetLinearVelocity(-3, 3);
    }
  }
  else{
    // If there, get new point
    if(dist_err < 0.05){
      ApplyTwist(0., 0.);
      perturbation_counter = 0;
      wait_counter = 0;
      return true;
    }

    else{
      // If not, drive there
      /* Calculate f_r (repulsive force) */
      CVector2 f_r;
      const CCI_KheperaIVProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

      for(size_t i = 0; i < tProxReads.size(); ++i) {
          f_r -= CVector2(tProxReads[i].Value, tProxReads[i].Angle);
      }

      f_r /= tProxReads.size();
      double d = KHEPERAIV_IR_SENSORS_RING_RANGE - f_r.Length();

      if (f_r.Length() > 0.) {
        f_r.Normalize();
        f_r.SetX(f_r.GetX() + m_pcRNG->Uniform(CRange(-0.1, 0.1)));
        f_r.SetY(f_r.GetY() + m_pcRNG->Uniform(CRange(-0.1, 0.1)));
      }

      f_r.Rotate(CRadians(yaw.GetValue()));

      /* Calculate f_a (attractive force) */
      CVector2 f_a(goal_pos.GetX() - curr_pos.GetX(), goal_pos.GetY() - curr_pos.GetY());
      if (f_a.Length() > 0.) { f_a.Normalize(); }

      double f = std::max(0., (-df/(d_max - d_min)) * (d - d_min) + df);
      CVector2 f_t = f_a + f * f_r;

      angle_err = atan2(f_t.GetY(), f_t.GetX()) - yaw.GetValue();
      if(angle_err >= M_PI) { angle_err -= 2 * M_PI; }
      if(angle_err < -M_PI) { angle_err += 2 * M_PI; }

      double omega_eff = theta_ctrl.computeEffort(angle_err);

      double v_des_coeff = std::max(0., std::min(1., (d - d_min) / (d_max - d_min)));
      // Clip it based on distance left
      v_des_coeff = std::min(v_des_coeff, std::max(dist_err, KHEPERAIV_BASE_RADIUS) / (KHEPERAIV_BASE_RADIUS));
      double v_err = maxRobotVelocity * v_des_coeff - curr_vel.Length();
      if (f_r.Length() > 0. && (angle_err >= M_PI / 2. || angle_err <= -M_PI / 2.)) { v_err *= -1.; }

      double vel_eff = vel_ctrl.computeEffort(v_err);

      ApplyTwist(vel_eff, omega_eff);
    }
  }
  return false;
}

//rotates around a point of specified radius
void CFootBotCollisionHandling::Circle(double radius){
  if(wait == true){
    m_pcWheels->SetLinearVelocity(0,0);
  }
  else{
    m_pcWheels->SetLinearVelocity(3*m_fWheelVelocity*(radius - wheelbase/2), 3*m_fWheelVelocity*(radius + wheelbase/2));
  }

  theta += 1;

  //if done circling
  if(abs(ogPos.GetX()-curr_pos[0]) < 0.06 && abs(ogPos.GetY()-curr_pos[1]) < 0.06 && theta > 50){
    if(curr_pos.GetX()-depot_x > 0.06 && curr_pos.GetY()-depot_y > 0.06){
      goal_pos.SetX(depot_x);
      goal_pos.SetY(depot_y);

      RotateDriveRotate(false);
    }
    else{
      theta = 0;
      m_eState = STATE_NEW_POINT;
    }
  }
}

void CFootBotCollisionHandling::ApplyTwist(double v_eff, double omega_eff) {
    double left_wheel_vel = v_eff - (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    double right_wheel_vel = v_eff + (KHEPERAIV_HALF_WHEEL_DISTANCE) * omega_eff;
    m_pcWheels->SetLinearVelocity(left_wheel_vel, right_wheel_vel);
}
/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotCollisionHandling, "collision_handling_controller")
