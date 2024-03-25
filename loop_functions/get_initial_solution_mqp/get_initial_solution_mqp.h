/*
 * This example shows how to define custom distributions to place the robots.
 */


#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>

#include "loop_functions/get_initial_solution_mqp/mqp_http_client.h"
#include <argos3/core/utility/configuration/argos_configuration.h>

using namespace argos;

class GetInitialSolutionMQP : public CLoopFunctions {

public:

    GetInitialSolutionMQP();
    virtual ~GetInitialSolutionMQP();

    virtual void Init(TConfigurationNode& t_tree);

    [[nodiscard]] inline const std::vector<std::vector<std::vector<std::vector<float>>>>& GetPath() const {
        return path_arr;
    }

private:
    std::vector<std::vector<std::vector<std::vector<float>>>> path_arr;

};

