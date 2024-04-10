//
// Created by Yaşar İdikut on 4/1/24.
//

#include "kheperaiv_orca_mqp_loop.h"

CKheperaIVORCAMQPLoop::CKheperaIVORCAMQPLoop() {

}

void CKheperaIVORCAMQPLoop::Init(TConfigurationNode& t_tree) {

    RequestPath(t_tree);

    auto obstacles = std::vector<std::vector<RVO::Vector2>>();
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

    double rab_range = 1.0;

    CQuaternion random_quat;
    random_quat.FromEulerAngles(CRadians(0.), CRadians(0.), CRadians(0.));
    auto m_pcRNG = CRandom::CreateRNG("argos");

    double delta_1 = 1.;
    double delta_2 = 0.65;
    double center_x = delta_1 / 2.;
    double center_y = delta_2 / 2. * 4;
    for (unsigned long i = 0; i < 2; ++i) {
        for (unsigned long j = 0; j < 5; ++j) {
            unsigned long robot_id = i * 5 + j;
//            std::cout << "New robot_id:" << robot_id << std::endl;

//            if (robot_id != 0) {
                random_quat.FromEulerAngles(m_pcRNG->Uniform(CRange(CRadians(-M_PI), CRadians(M_PI))), CRadians(0.), CRadians(0.));
//            }

            // Populate the robots array and configure the robot
            cKheperaIVs.push_back(new CKheperaIVEntity(
                    fmt::format("kp{}", robot_id),
                    "kheperaiv_orca_mqp_controller",
                    CVector3(center_x- i * delta_1, center_y - j * delta_2, 0),
                    random_quat,
                    rab_range));
            AddEntity(*cKheperaIVs[robot_id]);

            auto &cController = dynamic_cast<CKheperaIVORCAMQP &>(cKheperaIVs[robot_id]->GetControllableEntity().GetController());
            cController.id = robot_id;
            int goal_i = i == 0 ? 1 : 0;
            cController.goal_pos = CVector2(center_x - goal_i * delta_1, center_y - j * delta_2);
            cController.obstacles = obstacles;
            cController.rab_range = rab_range;

//            std::cout << "curr_pos:" << center_x- i * delta_1 << "," << center_y - j * delta_2 << std::endl;
//            std::cout << "goal_pos:" << cController.goal_pos.GetX() << "," << cController.goal_pos.GetY() << std::endl;

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
    mqp_http_client::printPaths(path_arr);

}


REGISTER_LOOP_FUNCTIONS(CKheperaIVORCAMQPLoop, "kheperaiv_orca_mqp_loop")

