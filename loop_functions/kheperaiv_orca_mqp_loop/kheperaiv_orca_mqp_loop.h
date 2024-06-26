#ifndef KHEPERA_ORCA_MQP_LOOP_H
#define KHEPERA_ORCA_MQP_LOOP_H

/*
 * Include some necessary headers.
 */
#include <argos3/core/simulator/loop_functions.h>

#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

#include <controllers/kheperaiv_orca_mqp/kheperaiv_orca_mqp.h>
#include "mqp_http_client.h"

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CKheperaIVORCAMQPLoop : public CLoopFunctions {

public:
    CKheperaIVORCAMQPLoop();
    virtual ~CKheperaIVORCAMQPLoop() {}

    virtual void Init(TConfigurationNode& t_node);
    virtual void PreStep();

    void RequestPath(TConfigurationNode& t_tree);
    void CalculateObstacles();

    std::vector<std::vector<std::vector<std::vector<double>>>> path_arr;
    std::vector<std::vector<RVO::Vector2>> obstacles;
    std::vector<CKheperaIVEntity*> cKheperaIVs;
    std::vector<double> depot;
    unsigned int depot_turn_robot_id = 0;
    unsigned long num_of_robots;

};

#endif
