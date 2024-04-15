//
// Created by Yaşar İdikut on 4/1/24.
//

#include "kheperaiv_orca_loop.h"

CKheperaIVORCALoop::CKheperaIVORCALoop() {

}

void CKheperaIVORCALoop::Init(TConfigurationNode& t_node) {

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
                    "kheperaiv_orca_controller",
                    CVector3(center_x- i * delta_1, center_y - j * delta_2, 0),
                    random_quat,
                    rab_range));
            AddEntity(*cKheperaIVs[robot_id]);

            auto &cController = dynamic_cast<CKheperaIVORCA &>(cKheperaIVs[robot_id]->GetControllableEntity().GetController());
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



void CKheperaIVORCALoop::PreStep() {
   std::string line = "";
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("kheperaiv");

   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {

      CKheperaIVEntity& cFootBot = *any_cast<CKheperaIVEntity*>(it->second);
      CKheperaIVORCA &cController = dynamic_cast<CKheperaIVORCA &>(cFootBot.GetControllableEntity().GetController());

      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

       line = line + std::to_string(cPos.GetX()) + "," + std::to_string(cPos.GetY()) + "," + std::to_string(cController.stepsInWorld) + ";";

       cController.stepsInWorld += 1;
   }

   x += 1;


   unsigned long robot_id = 0;
   unsigned long new_robot_id = 5;

   //if(x > (rand() % 10000) && onlyOneFailure != 1){ //failure happens

   if(x > 100000 && onlyOneFailure != 1){ //failure happens
     onlyOneFailure = 1;
     LOGERR << "Failure happens" << std::endl;
     k = k-1;

     RemoveEntity("kp"+std::to_string(rand() % k));

     //mqp_http_client::solve(&path_arr, host, k, n_a, fcr, fr, ssd, "h2", rp); //convert to recalculate

   }

   data_parsing::WriteLine(line);

}

REGISTER_LOOP_FUNCTIONS(CKheperaIVORCALoop, "kheperaiv_orca_loop")
