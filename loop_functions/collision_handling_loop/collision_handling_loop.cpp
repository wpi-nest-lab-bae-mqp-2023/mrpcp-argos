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

//    unsigned long num_of_robots = 3;
  num_of_robots = path_arr.size();
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
                  fmt::format("ch-{}", robot_id),
                  "ch",
                  CVector3(depot_x - i * delta, depot_y - j * delta, 0),
                  random_quat));
          AddEntity(*cKheperaIVs[robot_id]);

          auto &cController = dynamic_cast<CFootBotCollisionHandling &>(cKheperaIVs[robot_id]->GetControllableEntity().GetController());
          cController.id = robot_id;
          cController.SetPath(path_arr[robot_id]);

          cController.depot_x = depot_x;
          cController.depot_y = depot_y;
      }
  }

  std::cout << "Ran init in get_initial_solution_mqp.cpp" << std::endl;
}


void CollisionHandlingLoop::PreStep() {
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("kheperaiv");
   CVector2 robot_posn[num_of_robots];
   CRadians robot_angle[num_of_robots];

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

      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

       CRadians cZAngle, _;
       cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, _, _);

       for(int j = 0; j < num_of_robots; j++){
         if(cPos != robot_posn[j]){
           //cController.wait = false;

           double diffAngle = cZAngle.GetValue() - atan2(robot_posn[j].GetY() - cPos.GetY(), robot_posn[j].GetX() - cPos.GetX());
           while(diffAngle >= pi){
             diffAngle -= 2*pi;
           }
           while(diffAngle <= -pi){
             diffAngle += 2*pi;
           }

           double distance = sqrt(pow((robot_posn[j].GetY()-cPos.GetY()), 2) + pow((robot_posn[j].GetX()-cPos.GetX()), 2));

           if(diffAngle < 0.5 && diffAngle > -0.5 && distance < 0.5 && distance > -0.5){
             cController.wait = true;
           }
           //}
         }
       }
   }

}
/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CollisionHandlingLoop, "collision_handling_loop");
