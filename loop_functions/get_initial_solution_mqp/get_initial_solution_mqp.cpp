#include "get_initial_solution_mqp.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_mqp/footbot_mqp.h>

#include <sstream>
#include <list>

using namespace argos;



GetInitialSolutionMQP::GetInitialSolutionMQP() {
}

/****************************************/
/****************************************/

GetInitialSolutionMQP::~GetInitialSolutionMQP() {
}

/****************************************/
/****************************************/

void GetInitialSolutionMQP::Init(TConfigurationNode& t_tree) {
    std::cout << "Setting up in get_initial_solution_mqp.cpp" << std::endl;
    mqp_http_client::solve(&path_arr, "http://127.0.0.1:5000");
//    mqp_http_client::printPaths(path_arr);

    /* Check whether a robot is on a food item */
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

    unsigned int ki = 0;
    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
        it != m_cFootbots.end();
        ++it) {
        /* Get handle to foot-bot entity and controller */
        CFootBotEntity &cFootBot = *any_cast<CFootBotEntity *>(it->second);
        CFootBotMQP &cController = dynamic_cast<CFootBotMQP &>(cFootBot.GetControllableEntity().GetController());

        std::cout << "Robot id: " << ki << std::endl;
        cController.path_arr = path_arr[ki];
        mqp_http_client::printPath(cController.path_arr);
        ki += 1;
    }


    std::cout << "Ran init in get_initial_solution_mqp.cpp" << std::endl;
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(GetInitialSolutionMQP, "get_initial_solution_mqp");
