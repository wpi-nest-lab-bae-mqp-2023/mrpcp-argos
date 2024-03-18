#include "get_initial_solution_mqp.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

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
    mqp_http_client::printPath(path_arr);
    std::cout << "Ran init in get_initial_solution_mqp.cpp" << std::endl;
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(GetInitialSolutionMQP, "get_initial_solution_mqp");
