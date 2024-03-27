#include "get_initial_solution_mqp.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <controllers/kheperaiv_mqp/kheperaiv_mqp.h>

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

    std::string host; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "host", host, host);
    int k = 0; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "k", k, k);
    if (k <= 0) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (k<=0): Select k > 0."); }
    float nk = 0.; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "nk", nk, nk);
    if (nk <= 0.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (nk<=0): Select nk > 0."); }
    float fcr = 0.; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "fcr", fcr, fcr);
    if (fcr <= 1.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (fcr<=1): Select fcr > 1."); }
    float fr = 0.; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "fr", fr, fr);
    if (fr < 0.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (fr<0): Select fr >= 0."); }
    float ssd = 0.; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "ssd", ssd, ssd);
    if (ssd <= 0.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (ssd<=0): Select ssd > 0."); }
    std::string mode; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "mode", mode, mode);
    if (!(mode == "m" || mode == "h1" || mode == "h2")) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (mode!=m,h1,h2): Select mode as either 'm', 'h1', or 'h2'"); }

    std::cout << "Problem Specification Parameters:" << std::endl;
    std::cout << "\thost (problem solver server host): " << host << std::endl;
    std::cout << "\tk (number of robots): " << k << std::endl;
    std::cout << "\tnk (number of nodes in an axis per robot): " << nk << std::endl;
    std::cout << "\tfcr (fuel-capacity-ratio relative to minimum needed): " << fcr << std::endl;
    std::cout << "\tfr (failure-ratio relative ...): " << fr << std::endl;
    std::cout << "\tssd (square-side-distance in meters): " << ssd << std::endl;
    std::cout << "Waiting on a solution..." << std::endl;

    mqp_http_client::solve(&path_arr, host, k, nk, fcr, fr, ssd, mode);
//    mqp_http_client::printPaths(path_arr);

    CSpace::TMapPerType& m_ckheperaivs = GetSpace().GetEntitiesByType("kheperaiv");

    unsigned int ki = 0;
    for(CSpace::TMapPerType::iterator it = m_ckheperaivs.begin();
        it != m_ckheperaivs.end();
        ++it) {
        CKheperaIVEntity &cKheperaIV = *any_cast<CKheperaIVEntity *>(it->second);
        CKheperaIVMQP &cController = dynamic_cast<CKheperaIVMQP &>(cKheperaIV.GetControllableEntity().GetController());

        std::cout << "Robot id: " << ki << std::endl;
        cController.id = ki;
        cController.SetPath(path_arr[ki]);
        mqp_http_client::printPath(cController.path_arr);
        ki += 1;
    }


    std::cout << "Ran init in get_initial_solution_mqp.cpp" << std::endl;
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(GetInitialSolutionMQP, "get_initial_solution_mqp");
