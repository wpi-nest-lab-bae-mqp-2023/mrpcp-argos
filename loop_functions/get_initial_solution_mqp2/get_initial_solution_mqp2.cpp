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

    std::cout << "Ran init in get_initial_solution_mqp.cpp" << std::endl;
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

      //check if anyone is in the depot @ -1 -1
      //if y, WAIT[k] = True

      //check if anyone is circling before the current robot. if yes, slow speed

      robot_posn[ki] = cPos;
      ki += 1;
   }

   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {

      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotMQP2 &cController = dynamic_cast<CFootBotMQP2 &>(cFootBot.GetControllableEntity().GetController());

      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

      if(cController.m_eState == 1 && cController.path_arr[ki][0] == -1 && cController.path_arr[ki][1] == -1){//if current robot is entering depot
        //if(cPos.GetX() - )


      }

      for(int i = 0; i < numOfRobots; ++i){
        cController.robot_posn[i] = robot_posn[i];
      }
   }

}
/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(GetInitialSolutionMQP2, "get_initial_solution_mqp2");
