#include "collision_handling_loop.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <controllers/collision_handling/collision_mqp.h>

#include <sstream>

#include <list>
#include <fmt/core.h>

using namespace argos;

CollisionHandlingLoop::CollisionHandlingLoop() {
}

/****************************************/
/****************************************/

CollisionHandlingLoop::~CollisionHandlingLoop() {
}

/****************************************/
/****************************************/

void CollisionHandlingLoop::Init(TConfigurationNode& t_tree) {
  std::cout << "Setting up in get_initial_solution_mqp.cpp" << std::endl;

  GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "host", host, host);
  GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "k", k, k);
  if (k <= 0) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (k<=0): Select k > 0."); }
  GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "n_a", n_a, n_a);
  if (n_a <= 0.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (n_a<=0): Select n_a > 0."); }
  GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "fcr", fcr, fcr);
  if (fcr <= 1.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (fcr<=1): Select fcr > 1."); }
  GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "fr", fr, fr);
  if (fr < 0.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (fr<0): Select fr >= 0."); }
  GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "ssd", ssd, ssd);
  if (ssd <= 0.) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (ssd<=0): Select ssd > 0."); }
  GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "mode", mode, mode);

  GetNodeAttributeOrDefault(GetNode(t_tree, "problem_params"), "rp", rp, rp);

  if (!(mode == "m" || mode == "h1" || mode == "h2")) { THROW_ARGOSEXCEPTION("Incorrect/Incomplete Problem Parameter Specification (mode!=m,h1,h2): Select mode as either 'm', 'h1', or 'h2'"); }

  original_k = k;

  std::cout << "Problem Specification Parameters:" << std::endl;
  std::cout << "\thost (problem solver server host): " << host << std::endl;
  std::cout << "\tk (number of robots): " << k << std::endl;
  std::cout << "\tn_a (number of nodes in an axis per robot): " << n_a << std::endl;
  std::cout << "\tfcr (fuel-capacity-ratio relative to minimum needed): " << fcr << std::endl;
  std::cout << "\tfr (failure-ratio relative ...): " << fr << std::endl;
  std::cout << "\tssd (square-side-distance in meters): " << ssd << std::endl;
  std::cout << "Waiting on a solution..." << std::endl;

  mqp_http_client::solve(&path_arr, host, k, n_a, fcr, fr, ssd, mode, rp);
//    mqp_http_client::printPaths(path_arr);

//    unsigned long num_of_robots = 3;
  num_of_robots = path_arr.size();
  unsigned long num_of_robots_per_side = num_of_robots;

  CQuaternion random_quat;
  auto m_pcRNG = CRandom::CreateRNG("argos");

  double d_x = path_arr[0][0][0][0];
  double d_y = path_arr[0][0][0][1];
  double delta = 0.3; GetNodeAttributeOrDefault(GetNode(t_tree, "arena_params"), "initial-robot-spacing", delta, delta);

  for (unsigned long i = 0; i < num_of_robots_per_side; ++i) {
      for (unsigned long j = 0; j < num_of_robots_per_side; ++j) {
          unsigned long robot_id = i * num_of_robots_per_side + j;
          if (robot_id >= num_of_robots) { break; }

          random_quat.FromEulerAngles(CRadians(-M_PI), CRadians(0.), CRadians(0.));

          // Populate the robots array and configure the robot
          /*
          cKheperaIVs.push_back(new CKheperaIVEntity(
                  fmt::format("ch-{}", robot_id),
                  "ch",
                  CVector3(d_x - i+j * delta, -1.7, 0),
                  random_quat));*/


          cKheperaIVs.push_back(new CKheperaIVEntity(
                  fmt::format("ch-{}", robot_id),
                  "ch",
                  CVector3(3 - j * delta, -2 - i * delta, 0),
                  random_quat));

          /*
          cKheperaIVs.push_back(new CKheperaIVEntity(
                  fmt::format("ch-{}", robot_id),
                  "ch",
                  CVector3(-1 - j * delta, -2 - i * delta, 0),
                  random_quat));*/

          AddEntity(*cKheperaIVs[robot_id]);


          auto &cController = dynamic_cast<CFootBotCollisionHandling &>(cKheperaIVs[robot_id]->GetControllableEntity().GetController());
          cController.id = robot_id;
          cController.SetPath(path_arr[robot_id]);

          cController.depot_x = d_x;
          cController.depot_y = d_y;

          //cController.depot_arr = depot_arr;
      }
  }

  std::cout << "Ran init in get_initial_solution_mqp.cpp" << std::endl;
}

void CollisionHandlingLoop::PreStep() {
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("kheperaiv");
   CVector2 robot_posn[num_of_robots];
   CRadians robot_angle[num_of_robots];

   std::string line = "";

   int ki = 0;
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
      /* Get handle to foot-bot entity and controller */
      CKheperaIVEntity& cFootBot = *any_cast<CKheperaIVEntity*>(it->second);
      CFootBotCollisionHandling &cController = dynamic_cast<CFootBotCollisionHandling &>(cFootBot.GetControllableEntity().GetController());

      /* Get the position of the foot-bot on the ground as a CVector2 */
      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

      CRadians cZAngle, _;
      cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, _, _);

      robot_posn[ki] = cPos;
      robot_angle[ki] = cZAngle;

      ki += 1;
   }

   double pi = 3.1415926;

   //check if within the depot
   //check if distance between is greater than a certain thing and if one is oriented at another

   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {

      CKheperaIVEntity& cFootBot = *any_cast<CKheperaIVEntity*>(it->second);
      CFootBotCollisionHandling &cController = dynamic_cast<CFootBotCollisionHandling &>(cFootBot.GetControllableEntity().GetController());

      cController.wait = false;
      cController.willCollide = false;
      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

       CRadians cZAngle, _;
       cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, _, _);

       for(int j = 0; j < num_of_robots; j++){
         if(cPos != robot_posn[j]){

           double diffAngle = cZAngle.GetValue() - atan2(robot_posn[j].GetY() - cPos.GetY(), robot_posn[j].GetX() - cPos.GetX());
           double secondDiffAngle = robot_angle[j].GetValue() - atan2(cPos.GetY() - robot_posn[j].GetY(), cPos.GetX() - robot_posn[j].GetX());

           while(diffAngle >= pi){
             diffAngle -= 2*pi;
           }
           while(diffAngle <= -pi){
             diffAngle += 2*pi;
           }

           while(secondDiffAngle >= pi){
             secondDiffAngle -= 2*pi;
           }
           while(secondDiffAngle <= -pi){
             secondDiffAngle += 2*pi;
           }

           double distance = sqrt(pow((robot_posn[j].GetY()-cPos.GetY()), 2) + pow((robot_posn[j].GetX()-cPos.GetX()), 2));
           //if(abs(cPos.GetX() - (depot_x - 0.5)) < 2 && abs(cPos.GetY() - (depot_y - 0.5)) < 2){
           if(abs(diffAngle-secondDiffAngle) < 0.15 && abs(distance) < 0.35){
             cController.willCollide = true;
           }

            if(abs(diffAngle) < 0.6 && abs(distance) < 0.4){
             cController.wait = true;
            }
           }
         //}
       }
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

     RemoveEntity("ch-"+std::to_string(rand() % num_of_robots));

     mqp_http_client::solve(&path_arr, host, k, n_a, fcr, fr, ssd, "h2", rp); //convert to recalculate

   }

   data_parsing::WriteLine(line);
   /*
   if(x % 200 == 0){ //failure happens
     LOGERR << "Failure happens" << std::endl;
     k = k+1;

     CQuaternion quat;
     quat.FromEulerAngles(CRadians(0.), CRadians(0.), CRadians(0.));

     failedKheperaIVs.push_back(new CKheperaIVEntity(
         fmt::format("ch-{}", new_robot_id),
         "ch",
         CVector3(depot_x, depot_y, 0), quat));

     AddEntity(*failedKheperaIVs[new_robot_id]);

     auto &cController = dynamic_cast<CFootBotCollisionHandling &>(failedKheperaIVs[new_robot_id]->GetControllableEntity().GetController());
     cController.id = new_robot_id;
     cController.SetPath(path_arr[new_robot_id]);

     cController.depot_x = depot_x;
     cController.depot_y = depot_y;

     LOGERR << path_arr.size() << std::endl;
     mqp_http_client::solve(&path_arr, host, k, n_a, fcr, fr, ssd, "h2"); //convert to recalculate
     LOGERR << path_arr.size() << std::endl;

   }*/


}

//sending rq to recalculate function
//mark when MILP is being used, and when heuristic is being used, show on % coverage
//mark when robots fail on %coverage

//milp_paths
//heuristic
//milp_paths

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CollisionHandlingLoop, "collision_handling_loop");
