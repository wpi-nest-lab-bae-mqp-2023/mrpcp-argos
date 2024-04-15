#include "data_parsing.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <controllers/collision_handling/collision_mqp.h>

#include <sstream>
#include <iostream>

#include <list>

void data_parsing::WriteLine(std::string formattedRobotPositions){

  std::ofstream oss;
  oss.open("current_pos.txt", std::ios::app);
  oss << formattedRobotPositions << std::endl;

}
