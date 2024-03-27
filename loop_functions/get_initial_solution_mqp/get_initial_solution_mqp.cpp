#include "get_initial_solution_mqp.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <controllers/kheperaiv_mqp/kheperaiv_mqp.h>

#include <sstream>
#include <list>
#include <fmt/core.h>


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

    unsigned long num_of_robots = 3;
//    unsigned long num_of_robots = path_arr.size();
    unsigned long num_of_robots_per_side = std::ceil(std::sqrt((double)num_of_robots / 2.));

    CQuaternion random_quat;
    auto m_pcRNG = CRandom::CreateRNG("argos");

    double depot_x = path_arr[0][0][0][0];
    double depot_y = path_arr[0][0][0][1];
    double delta = 0.3; GetNodeAttributeOrDefault(GetNode(t_tree, "arena_params"), "initial-robot-spacing", delta, delta);

    for (unsigned long i = 0; i < num_of_robots_per_side; ++i) {
        for (unsigned long j = 0; j < num_of_robots_per_side; ++j) {
            unsigned long robot_id = i * num_of_robots_per_side + j;
            if (robot_id >= num_of_robots) { break; }

            random_quat.FromEulerAngles(m_pcRNG->Uniform(CRange(CRadians(-M_PI), CRadians(M_PI))), CRadians(0.), CRadians(0.));

            // Populate the robots array and configure the robot
            cKheperaIVs.push_back(new CKheperaIVEntity(
                    fmt::format("kheperaiv-{}", robot_id),
                    "kheperaiv_mqp_controller",
                    CVector3(depot_x - i * delta, depot_y - j * delta, 0),
                    random_quat));
            AddEntity(*cKheperaIVs[robot_id]);

            auto &cController = dynamic_cast<CKheperaIVMQP &>(cKheperaIVs[robot_id]->GetControllableEntity().GetController());
            cController.id = robot_id;
            cController.SetPath(path_arr[robot_id]);
        }
    }

    std::cout << "Ran init in get_initial_solution_mqp.cpp" << std::endl;
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(GetInitialSolutionMQP, "get_initial_solution_mqp");
