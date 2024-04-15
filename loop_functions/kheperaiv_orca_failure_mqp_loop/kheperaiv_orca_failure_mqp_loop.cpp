//
// Created by Yaşar İdikut on 4/1/24.
//

#include "kheperaiv_orca_failure_mqp_loop.h"

CKheperaIVORCAMQPLoop::CKheperaIVORCAMQPLoop() {

}

void CKheperaIVORCAMQPLoop::Init(TConfigurationNode& t_tree) {

    RequestPath(t_tree);
    CalculateObstacles();

    double rab_range = 1.0;

    CQuaternion random_quat;
    random_quat.FromEulerAngles(CRadians(0.), CRadians(0.), CRadians(0.));
    auto m_pcRNG = CRandom::CreateRNG("argos");

    //    unsigned long num_of_robots = 3;
    num_of_robots = path_arr.size();
    unsigned long num_of_robots_per_side = std::ceil(std::sqrt((double)num_of_robots));

//    double depot_x = path_arr[0][0][0][0];
//    double depot_y = path_arr[0][0][0][1];
    depot = path_arr[0][0][0];
    double delta = 0.3; GetNodeAttributeOrDefault(GetNode(t_tree, "arena_params"), "initial-robot-spacing", delta, delta);
    float fr = 0.; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "fr", fr, fr);
    if (fr > 1.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (fr>1): Select 0. >= fr >= 1."); }

    for (unsigned long i = 0; i < num_of_robots_per_side; ++i) {
        for (unsigned long j = 0; j < num_of_robots_per_side; ++j) {
            unsigned long robot_id = i * num_of_robots_per_side + j;
            if (robot_id >= num_of_robots) { break; }

            random_quat.FromEulerAngles(m_pcRNG->Uniform(CRange(CRadians(-M_PI), CRadians(M_PI))), CRadians(0.), CRadians(0.));

            // Populate the robots array and configure the robot
            cKheperaIVs.push_back(new CKheperaIVEntity(
                    fmt::format("kp{}", robot_id),
                    "kheperaiv_orca_failure_mqp_controller",
                    CVector3(depot[0] - i * delta, depot[1] - j * delta, 0),
                    random_quat,
                    rab_range));
            AddEntity(*cKheperaIVs[robot_id]);

            auto &cController = dynamic_cast<CKheperaIVORCAFailureMQP &>(cKheperaIVs[robot_id]->GetControllableEntity().GetController());
            cController.id = robot_id;
            cController.fr = fr;
            cController.obstacles = obstacles;
            cController.rab_range = rab_range;
            cController.SetPath(path_arr[robot_id]);
        }
    }

    std::cout << "Ran init in get_initial_solution_mqp.cpp" << std::endl;
}

void CKheperaIVORCAMQPLoop::PreStep() {
//    std::cout << "depot_turn_robot_id" << depot_turn_robot_id << std::endl;

//    for (int i = 0; i < num_of_robots; ++i) {
////        CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
//
//        auto cKheperaIV = &dynamic_cast<CKheperaIVEntity &>(GetSpace().GetEntity(fmt::format("kp{}", i)));
//        auto &cController = dynamic_cast<CKheperaIVORCAMQP &>(cKheperaIV->GetControllableEntity().GetController());
//        if (cController.id == depot_turn_robot_id) {
//            cController.is_turn_to_startup_depot = true;
//        }
////        std::cout << "id" << cController.id << "X: " << cController.goal_pos.GetX() << "; Y: " << cController.goal_pos.GetY() << std::endl;
//        if (cController.id == depot_turn_robot_id && cController.goal_pos.GetX() != depot[0] && cController.goal_pos.GetY() != depot[1]) {
//            depot_turn_robot_id += 1;
//        }
//    }

//    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("kheperaiv");
//
//    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
//        it != m_cFootbots.end();
//        ++it) {
//        /* Get handle to foot-bot entity and controller */
//        CFootBotEntity &cFootBot = *any_cast<CFootBotEntity *>(it->second);
//
//    }


    for (auto cKheperaIV : cKheperaIVs) {
        auto &cController = dynamic_cast<CKheperaIVORCAFailureMQP &>(cKheperaIV->GetControllableEntity().GetController());
        if (cController.id == depot_turn_robot_id) {
            cController.is_turn_to_startup_depot = true;
        }
//        std::cout << "id" << cController.id << "X: " << cController.goal_pos.GetX() << "; Y: " << cController.goal_pos.GetY() << std::endl;
        if (cController.id == depot_turn_robot_id && cController.goal_pos != cController.depot_pos && cController.did_leave_from_startup_depot) {
            depot_turn_robot_id += 1;
        }

    }
}



void CKheperaIVORCAMQPLoop::RequestPath(TConfigurationNode& t_tree) {
    std::cout << "Setting up in get_initial_solution_mqp.cpp" << std::endl;

    std::string host; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "host", host, host);
    int k = 0; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "k", k, k);
    if (k <= 0) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (k<=0): Select k > 0."); }
    float n_a = 0.; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "n_a", n_a, n_a);
    if (n_a <= 0.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (n_a<=0): Select n_a > 0."); }
    float fcr = 0.; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "fcr", fcr, fcr);
    if (fcr <= 1.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (fcr<=1): Select fcr > 1."); }
    float rp = 0.; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "rp", rp, rp);
    if (rp < 0.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (rp<0): Select rp >= 0."); }
    float ssd = 0.; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "ssd", ssd, ssd);
    if (ssd <= 0.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (ssd<=0): Select ssd > 0."); }
    std::string mode; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "mode", mode, mode);
    if (!(mode == "m" || mode == "h1" || mode == "h2")) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (mode!=m,h1,h2): Select mode as either 'm', 'h1', or 'h2'"); }

    std::cout << "Problem Specification Parameters:" << std::endl;
    std::cout << "\thost (problem solver server host): " << host << std::endl;
    std::cout << "\tk (number of robots): " << k << std::endl;
    std::cout << "\tn_a (number of nodes in an axis): " << n_a << std::endl;
    std::cout << "\tfcr (fuel-capacity-ratio relative to minimum needed): " << fcr << std::endl;
    std::cout << "\trp (redundancy parameter): " << rp << std::endl;
    std::cout << "\tssd (square-side-distance in meters): " << ssd << std::endl;
    std::cout << "Waiting on a solution..." << std::endl;

    mqp_http_client::solve(&path_arr, host, k, n_a, fcr, rp, ssd, mode);
//    mqp_http_client::printPaths(path_arr);

}

void CKheperaIVORCAMQPLoop::CalculateObstacles() {
    CSpace::TMapPerType& tBoxMap = GetSpace().GetEntitiesByType("box");
    /* Go through them */
    for(CSpace::TMapPerType::iterator it = tBoxMap.begin();
        it != tBoxMap.end();
        ++it) {
        CBoxEntity& cBox = *any_cast<CBoxEntity*>(it->second);
//        std::cout << "Box: " << "MinCorner: " << cBox.GetEmbodiedEntity().GetBoundingBox().MinCorner << " -  MaxCorner: " << cBox.GetEmbodiedEntity().GetBoundingBox().MaxCorner << std::endl;

        auto obstacle = std::vector<RVO::Vector2>();
        obstacle.emplace_back((float)cBox.GetEmbodiedEntity().GetBoundingBox().MaxCorner.GetX(), (float)cBox.GetEmbodiedEntity().GetBoundingBox().MaxCorner.GetY());
        obstacle.emplace_back((float)cBox.GetEmbodiedEntity().GetBoundingBox().MinCorner.GetX(), (float)cBox.GetEmbodiedEntity().GetBoundingBox().MaxCorner.GetY());
        obstacle.emplace_back((float)cBox.GetEmbodiedEntity().GetBoundingBox().MinCorner.GetX(), (float)cBox.GetEmbodiedEntity().GetBoundingBox().MinCorner.GetY());
        obstacle.emplace_back((float)cBox.GetEmbodiedEntity().GetBoundingBox().MaxCorner.GetX(), (float)cBox.GetEmbodiedEntity().GetBoundingBox().MinCorner.GetY());
        obstacles.push_back(obstacle);
    }
}



REGISTER_LOOP_FUNCTIONS(CKheperaIVORCAMQPLoop, "kheperaiv_orca_failure_mqp_loop")

