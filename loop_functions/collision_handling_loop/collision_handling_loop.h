/*
 * This example shows how to define custom distributions to place the robots.
 */


#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>

#include "loop_functions/collision_handling_loop/mqp_http_client.h"
#include <argos3/core/utility/configuration/argos_configuration.h>

using namespace argos;

class CollisionHandlingLoop : public CLoopFunctions {

public:

    CollisionHandlingLoop();
    virtual ~CollisionHandlingLoop();

    virtual void Init(TConfigurationNode& t_tree);
    virtual void PreStep();

    [[nodiscard]] inline const std::vector<std::vector<std::vector<std::vector<float>>>>& GetPath() const {
        return path_arr;
    }

private:
  std::vector<std::vector<std::vector<std::vector<float>>>> path_arr;
  std::vector<CKheperaIVEntity*> cKheperaIVs;

  unsigned long num_of_robots;

};
