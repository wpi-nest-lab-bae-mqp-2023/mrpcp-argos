/*
 * This example shows how to define custom distributions to place the robots.
 */

#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class MRPCPLoopFunctions : public CLoopFunctions {

public:

   MRPCPLoopFunctions();
   virtual ~MRPCPLoopFunctions();

   virtual void Init(TConfigurationNode& t_tree);
   static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);
};

