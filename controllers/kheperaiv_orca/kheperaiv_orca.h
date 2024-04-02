#ifndef KHEPERA_ORCA_H
#define KHEPERA_ORCA_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CKheperaIVORCA : public CCI_Controller {

public:
    CKheperaIVORCA();
    virtual ~CKheperaIVORCA() {}

    virtual void Init(TConfigurationNode& t_node);
    virtual void ControlStep();

    virtual void Reset();
    virtual void Destroy() {}

private:

    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;
};

#endif
