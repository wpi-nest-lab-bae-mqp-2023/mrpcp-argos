//
// Created by Yaşar İdikut on 4/1/24.
//

#include "kheperaiv_orca_failure_mqp_loop.h"


CKheperaIVORCAMQPLoop::CKheperaIVORCAMQPLoop() {

}


void CKheperaIVORCAMQPLoop::Init(TConfigurationNode& t_tree) {

    RequestPath(t_tree);
    CalculateObstacles();

    CQuaternion random_quat;
    random_quat.FromEulerAngles(CRadians(0.), CRadians(0.), CRadians(0.));
    auto m_pcRNG = CRandom::CreateRNG("argos");

//    num_of_robots = 3;
    num_of_robots = original_path_arr.size();
    num_of_robots_per_side = std::ceil(std::sqrt((double)num_of_robots));

    depot = original_path_arr[0][0][0];
    delta = 0.25; GetNodeAttributeOrDefault(GetNode(t_tree, "arena_params"), "initial-robot-spacing", delta, delta);
    fr = 0.; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "fr", fr, fr);
    if (fr > 1.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (fr>1): Select 0. >= fr >= 1."); }
    frt = 0.; GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "frt", frt, frt);
    if (frt < 0.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (frt<0): Select frt >= 0."); }
    string position_logging_output_folder = ""; GetNodeAttributeOrDefault(GetNode(t_tree, "position_logging_params"), "output-folder", position_logging_output_folder, position_logging_output_folder);

    if (GetSimulator().GetPhysicsEngines()[0]->GetId() == "dyn2d") {
        for (unsigned long i = 0; i < num_of_robots_per_side; ++i) {
            for (unsigned long j = 0; j < num_of_robots_per_side; ++j) {
                unsigned long robot_id = i * num_of_robots_per_side + j;
                if (robot_id >= num_of_robots) { break; }

                random_quat.FromEulerAngles(m_pcRNG->Uniform(CRange(CRadians(-M_PI), CRadians(M_PI))), CRadians(0.), CRadians(0.));

                // Populate the robots array and configure the robot
                auto cKheperaIV = new CKheperaIVEntity(
                        "kp" + std::to_string(robot_id),
                        "kheperaiv_orca_failure_mqp_controller",
                        CVector3(depot[0] - i * delta - depot_offset, depot[1] - j * delta - depot_offset, 0),
                        random_quat,
                        rab_range);
                AddEntity(*cKheperaIV);
            }
        }
    }

    pos_logger = new position_logger(position_logging_output_folder);
    std::cout << "Ran init in kheperaiv_orca_failure_mqp_loop.cpp" << std::endl;
}


void CKheperaIVORCAMQPLoop::PreStep() {
    updateKheperaIVs();
    // Start robots in order at start
    unsigned int depot_turn_robot_id = 0;
    for (auto cKheperaIV : cKheperaIVs) {
        auto &cController = dynamic_cast<CKheperaIVORCAFailureMQP &>(cKheperaIV->GetControllableEntity().GetController());
        if (cController.id == depot_turn_robot_id) {
            cController.is_turn_to_startup_depot = true;
        }
        if (cController.id == depot_turn_robot_id && cController.did_leave_from_startup_depot) {
//            std::cout << "id" << cController.id << " is done, letting next robot go..." << std::endl;
            depot_turn_robot_id += 1;
        }
    }

    // If a failure happens, respawn robots after some time period
    unsigned int num_of_failed_robots = cKheperaIVs.size();
    std::vector<unsigned int> healthy_robots = std::vector<unsigned int>();
    for (int ki = 0; ki < cKheperaIVs.size(); ++ki) {
        int i = floor(ki / num_of_robots_per_side);
        int j = ki % num_of_robots_per_side;
        auto cKheperaIV = cKheperaIVs[ki];
        auto &cController = dynamic_cast<CKheperaIVORCAFailureMQP &>(cKheperaIV->GetControllableEntity().GetController());
        if (cController.since_failed_counter == 0) {
            num_of_failed_robots -= 1;
            healthy_robots.push_back(ki);
        }
        if (cController.since_failed_counter >= frt * 10) {
            if (GetSimulator().GetPhysicsEngines()[0]->GetId() == "dyn2d") {
                // Teleport robot, but also delay if can't place to the start location
                bool success = MoveEntity(cKheperaIV->GetEmbodiedEntity(), CVector3(depot[0] - i * delta - depot_offset, depot[1] - j * delta - depot_offset, 0), CQuaternion().FromEulerAngles(CRadians(0.), CRadians(0.), CRadians(0.)));
                if (!success) { continue; }
            }
            cController.Reset();
            std::cout << "kp" << ki << " respawned!" << std::endl;
        }
    }

    if (healthy_robots != prev_healthy_robots && healthy_robots.size() == k && is_prev_path_recalc) {
        most_recent_path_arr = original_path_arr;
        unsigned int path_counter = 0;
        for (auto cKheperaIV : cKheperaIVs) {
            auto &cController = dynamic_cast<CKheperaIVORCAFailureMQP &>(cKheperaIV->GetControllableEntity().GetController());
            if (cController.since_failed_counter) { continue; }
            cController.SetPath(most_recent_path_arr[path_counter]);
            path_counter += 1;
        }
        is_prev_path_recalc = false;
    }
    // Recalculate paths if enough robots fail
    else if (healthy_robots != prev_healthy_robots && num_of_failed_robots >= rp) {
        // Construct the lists of healthy robots that can execute the recalculated paths
        std::string curr_robots_pos = "";
        std::string curr_fuel_levels = "";
        for (auto cKheperaIV : cKheperaIVs) {
            auto &cController = dynamic_cast<CKheperaIVORCAFailureMQP &>(cKheperaIV->GetControllableEntity().GetController());
            if (cController.since_failed_counter) { continue; }
            CVector2 curr_robot_pos = CVector2(cKheperaIV->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), cKheperaIV->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
            if (curr_robot_pos.GetX() < depot[0] && curr_robot_pos.GetY() < depot[1]) {
                curr_robot_pos.Set(depot[0], depot[1]);
            }
            curr_robots_pos = curr_robots_pos + "[" + std::to_string(curr_robot_pos.GetX()) + ", " + std::to_string(curr_robot_pos.GetY()) +"],";
            curr_fuel_levels = curr_fuel_levels + std::to_string(1.) +",";
        }
        curr_robots_pos = "[" + curr_robots_pos.substr(0, curr_robots_pos.size()-1) + "]";
        curr_fuel_levels = "[" + curr_fuel_levels.substr(0, curr_fuel_levels.size()-1) + "]";

        mqp_http_client::recalculate(&most_recent_path_arr, host, k-num_of_failed_robots, n_a, fcr, rp, ssd, mode, curr_fuel_levels, curr_robots_pos);
        unsigned int path_counter = 0;
        for (auto cKheperaIV : cKheperaIVs) {
            auto &cController = dynamic_cast<CKheperaIVORCAFailureMQP &>(cKheperaIV->GetControllableEntity().GetController());
            if (cController.since_failed_counter) { continue; }
            std::cout << "setting the path of " << cController.id << " path_counter" << path_counter << std::endl;
            bool is_turn_to_startup_depot = cController.is_turn_to_startup_depot;
            bool did_leave_from_startup_depot = cController.did_leave_from_startup_depot;
            cController.SetPath(most_recent_path_arr[path_counter]);
            cController.is_turn_to_startup_depot = is_turn_to_startup_depot;
            cController.did_leave_from_startup_depot = did_leave_from_startup_depot;
            path_counter += 1;
        }
        is_prev_path_recalc = true;
    }
    prev_healthy_robots = healthy_robots;

    // Log positions to an output file
    pos_logger->write_to_logs(cKheperaIVs);
}


void CKheperaIVORCAMQPLoop::updateKheperaIVs() {
    if (num_of_robots == cKheperaIVs.size()) { return; }

    CSpace::TMapPerType &m_cKheperaIVs = GetSpace().GetEntitiesByType("kheperaiv");

    for (CSpace::TMapPerType::iterator it = m_cKheperaIVs.begin();
         it != m_cKheperaIVs.end();
         ++it) {
        /* Get handle to foot-bot entity and controller */
        auto &cKheperaIV = any_cast<CKheperaIVEntity *>(it->second);
        auto &cController = dynamic_cast<CKheperaIVORCAFailureMQP &>(cKheperaIV->GetControllableEntity().GetController());

        bool exists = cController.path_arr.size() > 0;
        if (exists) { continue; }
        CVector2 new_robot_pos;
        new_robot_pos.Set(cKheperaIV->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                          cKheperaIV->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        double new_robot_depot_dist = (new_robot_pos - CVector2(depot[0], depot[1])).Length();

        unsigned int robot_id = cKheperaIVs.size();
        for (int ki = 0; ki < cKheperaIVs.size(); ++ki) {
            CVector2 existing_robot_pos;
            existing_robot_pos.Set(cKheperaIVs[ki]->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                                   cKheperaIVs[ki]->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
            double existing_robot_depot_dist = (existing_robot_pos - CVector2(depot[0], depot[1])).Length();
            if (existing_robot_depot_dist > new_robot_depot_dist) {
                robot_id = ki;
                break;
            }
        }
        cController.fr = fr;
        cController.obstacles = obstacles;
        cController.rab_range = rab_range;
        cController.SetPath(most_recent_path_arr[robot_id]);
        cKheperaIVs.insert(cKheperaIVs.begin() + robot_id, cKheperaIV);
    }
    for (int ki = 0; ki < cKheperaIVs.size(); ++ki) {
        auto &cController = dynamic_cast<CKheperaIVORCAFailureMQP &>(cKheperaIVs[ki]->GetControllableEntity().GetController());
        cController.id = ki;
    }
}


void CKheperaIVORCAMQPLoop::RequestPath(TConfigurationNode& t_tree) {
    std::cout << "Setting up in kheperaiv_orca_failure_mqp_loop.cpp" << std::endl;

    GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "host", host, host);
    GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "k", k, k);
    if (k <= 0) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (k<=0): Select k > 0."); }
    GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "n_a", n_a, n_a);
    if (n_a < 2) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (n_a<2): Select n_a >= 2"); }
    GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "fcr", fcr, fcr);
    if (fcr <= 1.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (fcr<=1): Select fcr > 1."); }
    GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "rp", rp, rp);
    if (rp < 1) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (rp<1): Select rp >= 1."); }
    GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "ssd", ssd, ssd);
    if (ssd <= 0.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (ssd<=0): Select ssd > 0."); }
    GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "mode", mode, mode);
    if (!(mode == "m" || mode == "h1" || mode == "h2")) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (mode!=m,h1,h2): Select mode as either 'm', 'h1', or 'h2'"); }

    double sr = ssd / (sqrt(2.) * n_a);  // surveillance radius
    ssd -= sqrt(2.) * sr; // Override ssd to put padding

    std::cout << "Problem Specification Parameters:" << std::endl;
    std::cout << "\thost (problem solver server host): " << host << std::endl;
    std::cout << "\tk (number of robots): " << k << std::endl;
    std::cout << "\tn_a (number of nodes in an axis): " << n_a << std::endl;
    std::cout << "\tfcr (fuel-capacity-ratio relative to minimum needed): " << fcr << std::endl;
    std::cout << "\trp (redundancy parameter): " << rp << std::endl;
    std::cout << "\tssd (square-side-distance in meters): " << ssd << std::endl;
    std::cout << "Waiting on a solution..." << std::endl;

    mqp_http_client::solve(&original_path_arr, host, k, n_a, fcr, rp, ssd, mode);
//    mqp_http_client::printPaths(original_path_arr);
    most_recent_path_arr = original_path_arr;
}


void CKheperaIVORCAMQPLoop::Reset() { }


void CKheperaIVORCAMQPLoop::CalculateObstacles() {
    CSpace::TMapPerType& tBoxMap = GetSpace().GetEntitiesByType("box");
    /* Go through them */
    for(CSpace::TMapPerType::iterator it = tBoxMap.begin();
        it != tBoxMap.end();
        ++it) {
        CBoxEntity& cBox = *any_cast<CBoxEntity*>(it->second);
//        std::cout << "Box: " << "MinCorner: " << cBox.GetEmbodiedEntity().GetBoundingBox().MinCorner << " -  MaxCorner: " << cBox.GetEmbodiedEntity().GetBoundingBox().MaxCorner << std::endl;

        auto obstacle = std::vector<RVO::Vector2>();
        obstacle.emplace_back((float)cBox.GetEmbodiedEntity().GetBoundingBox().MaxCorner.GetX() + KHEPERAIV_BASE_RADIUS / 2., (float)cBox.GetEmbodiedEntity().GetBoundingBox().MaxCorner.GetY() + KHEPERAIV_BASE_RADIUS / 2.);
        obstacle.emplace_back((float)cBox.GetEmbodiedEntity().GetBoundingBox().MinCorner.GetX() - KHEPERAIV_BASE_RADIUS / 2., (float)cBox.GetEmbodiedEntity().GetBoundingBox().MaxCorner.GetY() + KHEPERAIV_BASE_RADIUS / 2.);
        obstacle.emplace_back((float)cBox.GetEmbodiedEntity().GetBoundingBox().MinCorner.GetX() - KHEPERAIV_BASE_RADIUS / 2., (float)cBox.GetEmbodiedEntity().GetBoundingBox().MinCorner.GetY() - KHEPERAIV_BASE_RADIUS / 2.);
        obstacle.emplace_back((float)cBox.GetEmbodiedEntity().GetBoundingBox().MaxCorner.GetX() + KHEPERAIV_BASE_RADIUS / 2., (float)cBox.GetEmbodiedEntity().GetBoundingBox().MinCorner.GetY() - KHEPERAIV_BASE_RADIUS / 2.);
        obstacles.push_back(obstacle);
    }
}


REGISTER_LOOP_FUNCTIONS(CKheperaIVORCAMQPLoop, "kheperaiv_orca_failure_mqp_loop")
