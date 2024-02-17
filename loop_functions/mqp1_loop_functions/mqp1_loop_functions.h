/*
 * This example shows how to define custom distributions to place the robots.
 */

#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class MQP1LoopFunctions : public CLoopFunctions {

public:

   MQP1LoopFunctions();
   virtual ~MQP1LoopFunctions();

   virtual void Init(TConfigurationNode& t_tree);
   static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);
};

