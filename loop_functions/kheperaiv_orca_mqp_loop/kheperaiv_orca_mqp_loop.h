#ifndef KHEPERA_ORCA_MQP_LOOP_H
#define KHEPERA_ORCA_MQP_LOOP_H

/*
 * Include some necessary headers.
 */
#include <argos3/core/simulator/loop_functions.h>

#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <fmt/core.h>

#include <controllers/kheperaiv_orca_mqp/kheperaiv_orca_mqp.h>

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

    std::vector<CKheperaIVEntity*> cKheperaIVs;

};

#endif
