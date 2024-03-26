/* Include the controller definition */
#include "footbot_mqp2.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <argos3/core/utility/logging/argos_log.h>

#include <numbers>

/****************************************/
/****************************************/

CFootBotMQP2::CFootBotMQP2() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(20.0f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotMQP2::Init(TConfigurationNode& t_node) {
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
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );

   m_pcPosSens   = GetSensor  <CCI_PositioningSensor        >("positioning"       );
   /*Read
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them iReadn the config file so we don't
    * have to recompile if we want to try other settings.
    */

   GetNodeAttribute(t_node, "path", path);
   GetNodeAttribute(t_node, "path_length", path_length);
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   Reset();

}


void CFootBotMQP2::Reset(){
  pos = m_pcPosSens->GetReading().Position;
  quat = m_pcPosSens->GetReading().Orientation;

  //taking in information from xml about path

  std::string firstpath;
  std::string otherpaths;
  std::ifstream allpaths("path.txt");
  std::ofstream oss("temp.txt");

  //takes in first path, gives it to first robot, moves rest of paths to a temp file, and when ready,
  //copies over to the original file (excluding the first path) -- this also means you have to copy over
  //wherever you store your paths to the file "path.txt" every time you want to run (including if you reset)
  if(allpaths.is_open()){
    getline(allpaths,firstpath);

    otherpaths = firstpath.substr(firstpath.find(":")+1, firstpath.size());
    firstpath = firstpath.substr(0, firstpath.find(":")); // token is "scott"

    std::stringstream robots(firstpath);

    oss << otherpaths;

    std::string pathxandy;
    while(!robots.eof()){
      getline(robots,pathxandy,',');

      std::stringstream ws(pathxandy);
      std::string pathpoint;

      xorycount = 0;
      while(ws >> pathpoint){
        path_arr[countinstr][xorycount] = strtod(pathpoint.c_str(), NULL);
        xorycount += 1;
      }
      countinstr += 1;
    }
    robot_count += 1;
  }

  allpaths.close();
  rename("temp.txt", "path.txt");

  m_eState = STATE_GOING_TO_POINT;
}



/****************************************/
/****************************************/

void CFootBotMQP2::ControlStep() {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();

   pos = m_pcPosSens->GetReading().Position;
   quat = m_pcPosSens->GetReading().Orientation;
   quat.ToEulerAngles(yaw, temp1, temp2);

   /*
   m_cOutput.close();

   m_cOutput.open("positions.txt", std::ios_base::trunc | std::ios_base::out);
   m_cOutput << std::to_string(pos[1]) + "\t" << std::endl;
   */

   switch(m_eState) {
     case STATE_GOING_TO_POINT: {
       if(path_arr[c][0] == depot[0] && path_arr[c][1] == depot[1]){
         RotateDriveRotate(path_arr[c][0]-pos[0], path_arr[c][1]-pos[1], yaw, true);
       }
       else{
         RotateDriveRotate(path_arr[c][0]-pos[0], path_arr[c][1]-pos[1], yaw, false);
       }

      break;
     }
     case STATE_NEW_POINT: { //gets a new point from the .argos file -- currently set to just rotate through infinitely
       c += 1;
       if(c==path_length){
         c = 0;
       }
       LOGERR << "new point:" << path_arr[c][0] << "," << path_arr[c][1] << std::endl;

       m_eState = STATE_GOING_TO_POINT;
       break;
     }
     case AT_DEPOT: {
       Circle(0.5);
       break;
     }
     default: {
        LOGERR << "We can't be here, there's a bug!" << std::endl;
     }
  }
}

void CFootBotMQP2::RotateDriveRotate(double x, double y, argos::CRadians yaw, bool goingToDepot){
  //rotate to desired position
  /*
  if(finished_first_rot == false){
    finished_first_rot = Rotate(x, y, yaw);
  }
  else{ //when finished rotating, drive to desired position
      double dist = sqrt(pow(x,2)+pow(y,2));

      if(finished_drive == false){
        finished_drive = Drive(dist);
      }
      else{ //when finished driving, rotate to desired angle
        if(goingToDepot && finished_second_rot == false){
          finished_second_rot = Rotate(0, 1, yaw);
        }
        else {
          finished_drive = false;
          finished_first_rot = false;
          finished_second_rot = false;

          if(goingToDepot){
            m_eState = AT_DEPOT;
          }
          else{
            m_eState = STATE_NEW_POINT;
          }
        }
      }
  }
  */
  if(finished_first_rot == false){
    finished_first_rot = Rotate(x, y, yaw);
  }
  else{ //when finished rotating, drive to desired position
    double dist;
      if(goingToDepot == true){
        dist = sqrt(pow(x,2)+pow(y,2))-0.5;
      }
      else{
        dist = sqrt(pow(x,2)+pow(y,2));
      }

      if(finished_drive == false){
        finished_drive = Drive(dist);
        ogYaw.SetValue(yaw.GetValue() - 1.5708);
      }
      else{ //when finished driving, rotate to desired angle
        if(goingToDepot && finished_second_rot == false){
          if(goingToDepot == true){
            finished_second_rot = RotateToCircle(ogYaw, yaw);
          }
          else{
            finished_second_rot = Rotate(0, 1, yaw);
          }
        }
        else {
          finished_drive = false;
          finished_first_rot = false;
          finished_second_rot = false;

          if(goingToDepot){
            m_eState = AT_DEPOT;
          }
          else{
            m_eState = STATE_NEW_POINT;
          }
        }
      }
  }

}

//rotates to a set (x, y) position. yaw is the current orientation
bool CFootBotMQP2::Rotate(double x, double y, argos::CRadians yaw){
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
      return true;
    }
    return false;
}

bool CFootBotMQP2::RotateToCircle(argos::CRadians desiredAngle, argos::CRadians yaw){
    angleerr =  desiredAngle.GetValue() - yaw.GetValue();
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
      return true;
    }
    return false;
}


//drives a certain distance
bool CFootBotMQP2::Drive(double distance){
    if(wait == false && (distance > 0.05 || distance < -0.05)){
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }
    else if(wait == true){
      m_pcWheels->SetLinearVelocity(0, 0);
    }
    else {
      m_pcWheels->SetLinearVelocity(0, 0);
      return true;
    }
    return false;
}

//rotates around a point of specified radius
void CFootBotMQP2::Circle(double radius){
  m_pcWheels->SetLinearVelocity(3*m_fWheelVelocity*(radius - wheelbase/2), 3*m_fWheelVelocity*(radius + wheelbase/2));

  LOGERR << pos[0] << "<" << pos[1] << std::endl;
  //if done circling
  if(-0.5-pos[0] < 0.06 && -0.5-pos[0] > -0.06 && depot[1]-pos[1] < 0.06 && depot[1]-pos[1] > -0.06){
    m_eState = STATE_NEW_POINT;
  }
}

//if there is a robot within () of the depot, returns true, else returns false
//bool CFootBotDiffusion::RobotNearbyDepot(){

//}

/*
//returns posn of other robots in circle
void CFootBotDiffusion::OtherRobotsInCircle(){

}*/

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
REGISTER_CONTROLLER(CFootBotMQP2, "footbot_mqp2_controller")
