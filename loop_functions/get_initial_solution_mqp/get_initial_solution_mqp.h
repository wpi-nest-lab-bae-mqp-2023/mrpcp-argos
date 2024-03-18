/*
 * This example shows how to define custom distributions to place the robots.
 */

#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class GetInitialSolutionMQP : public CLoopFunctions {

public:

    GetInitialSolutionMQP();
   virtual ~GetInitialSolutionMQP();

   virtual void Init(TConfigurationNode& t_tree);
   static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);

private:
    std::vector<std::vector<std::vector<float>>> path_arr; //set to max # of nodes in possible space L/2

};

