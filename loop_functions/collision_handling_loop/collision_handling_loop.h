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
  std::vector<CKheperaIVEntity*> failedKheperaIVs;

  unsigned long num_of_robots;
  double depot_x;
  double depot_y;

  float lambda = 0.5;

  int original_k = 0;

  //problem parameters
  int x = 0;
  std::string host;
  int k = 0;
  float n_a = 0.;
  float fcr = 0.;
  float fr = 0.;
  float ssd = 0.;
  int rp = 0;
  std::string mode;

  double onlyOneFailure = 0;
};
