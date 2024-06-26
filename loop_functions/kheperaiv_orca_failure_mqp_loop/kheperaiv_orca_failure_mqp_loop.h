#ifndef KHEPERA_ORCA_FAILURE_MQP_LOOP_H
#define KHEPERA_ORCA_FAILURE_MQP_LOOP_H

/*
 * Include some necessary headers.
 */
#include <argos3/core/simulator/loop_functions.h>

#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

#include "controllers/kheperaiv_orca_failure_mqp/kheperaiv_orca_failure_mqp.h"
#include "mqp_http_client.h"
#include "position_logger.h"

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
    virtual void Reset();

    void RequestPath(TConfigurationNode& t_tree);
    void CalculateObstacles();
    void updateKheperaIVs();

    std::vector<std::vector<std::vector<std::vector<double>>>> original_path_arr;
    std::vector<std::vector<std::vector<std::vector<double>>>> most_recent_path_arr;
    std::vector<std::vector<RVO::Vector2>> obstacles;
    std::vector<CKheperaIVEntity*> cKheperaIVs;
    std::vector<double> depot;
    unsigned long num_of_robots;
    double delta;
    double depot_offset = 0.01;
    unsigned long num_of_robots_per_side;
    double frt;

    //problem parameters
    std::string host;
    unsigned int k = 0;
    unsigned int n_a = 0;
    float fcr = 0.;
    float fr = 0.;
    float ssd = 0.;
    unsigned int rp = 0;
    std::string mode;
    position_logger* pos_logger;

    double rab_range = 1.0;
    std::vector<unsigned int> prev_healthy_robots = std::vector<unsigned int>();
    bool is_prev_path_recalc = false;
    unsigned int lowest_ki = 0;

};

#endif
