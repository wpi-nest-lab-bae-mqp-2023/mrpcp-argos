#include "get_initial_solution_mqp2.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_mqp2/footbot_mqp2.h>

#include <sstream>
#include <list>


GetInitialSolutionMQP2::GetInitialSolutionMQP2() {
}

/****************************************/
/****************************************/

GetInitialSolutionMQP2::~GetInitialSolutionMQP2() {
}

/****************************************/
/****************************************/

void GetInitialSolutionMQP2::Init(TConfigurationNode& t_tree) {
    LOGERR << "Setting up in get_initial_solution_mqp.cpp" << std::endl;
    /*
    mqp_http_client::solve(&path_arr, "http://127.0.0.1:5000");
    mqp_http_client::printPaths(path_arr);

    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

    unsigned int ki = 0;
    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
        it != m_cFootbots.end();
        ++it) {
        CFootBotEntity &cFootBot = *any_cast<CFootBotEntity *>(it->second);
        CFootBotMQP2 &cController = dynamic_cast<CFootBotMQP2 &>(cFootBot.GetControllableEntity().GetController());

        std::cout << "Robot id: " << ki << std::endl;
        //cController.path_arr = path_arr[ki];
        //mqp_http_client::printPath(cController.path_arr);
        ki += 1;
    }
    */

    LOGERR << "Ran init in get_initial_solution_mqp.cpp" << std::endl;
}



void GetInitialSolutionMQP2::PreStep() {
    int numOfRobots = 2;
   /* Logic to pick and drop food items */
   /*
    * If a robot is in the nest, drop the food item
    * If a robot is on a food item, pick it
    * Each robot can carry only one food item per time
    */

   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
   CVector2 robot_posn[numOfRobots];
   CRadians robot_angle[numOfRobots];

   int ki = 0;
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotMQP2 &cController = dynamic_cast<CFootBotMQP2 &>(cFootBot.GetControllableEntity().GetController());

      /* Get the position of the foot-bot on the ground as a CVector2 */
      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

      CRadians cZAngle, _;
      cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, _, _);
      //check if anyone is in the depot @ -1 -1
      //if y, WAIT[k] = True

      //check if anyone is circling before the current robot. if yes, slow speed

      robot_posn[ki] = cPos;
      robot_angle[ki] = cZAngle;
      ki += 1;
   }


   //check if within the depot
   //check if distance between is greater than a certain thing and if one is oriented at another
   for(int i = 0; i < numOfRobots; i++){
   }

   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {

      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotMQP2 &cController = dynamic_cast<CFootBotMQP2 &>(cFootBot.GetControllableEntity().GetController());

      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

       CRadians cZAngle, _;
       cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, _, _);

       for(int j = 0; j < numOfRobots; j++){
         if(cPos != robot_posn[j]){
           cController.wait = false;

           double diffAngle = cZAngle.GetValue() - atan2(robot_posn[j].GetY() - cPos.GetY(), robot_posn[j].GetX() - cPos.GetX());
           double distance = sqrt(pow((robot_posn[j].GetY()-cPos.GetY()), 2) + pow((robot_posn[j].GetX()-cPos.GetX()), 2));
           if(diffAngle < 0.05 && diffAngle > -0.05 && distance < 0.5 && distance > -0.5){
             LOGERR <<"!" << std::endl;
             cController.wait = true;
           }
           //}
         }
       }
   }

}
/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(GetInitialSolutionMQP2, "get_initial_solution_mqp2");
